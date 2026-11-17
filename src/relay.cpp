/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

/*
 * relay 主循环实现：iceoryx2 主题装配 + 500Hz 转发节拍。
 *
 * 发布主题（relay → 任意进程）：imu 500Hz、chassis 100Hz、robot/game 1Hz、health 10Hz
 * 订阅主题（视觉/导航 → relay）：cmd_aim（云台瞄准指令）、cmd_nav（底盘速度指令）
 *
 * 节流策略：Imu 只在 CommBoard 解出新帧时发布，避免重复样本污染视觉侧的插值；
 *           chassis / robot / game / health 按固定周期发布，内容取最新快照。
 *           下行指令每拍排空队列，只执行最后一条——旧指令没有意义，执行它只会让云台晚到。
 *
 * 注意：契约见 relay.hpp，本文件只写实现要点。
 */
#include "src/relay.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>
#include <utility>

#include "iox2/iceoryx2.hpp"
#include "IO/inc/IO.hpp"
#include "IO/inc/board.hpp"
#include "msg/relay_msgs.hpp"
#include "tools/comm_warning.hpp"
#include "tools/logger.hpp"

namespace info_relay
{
namespace
{
// 功能：把 steady_clock 时间点转成纳秒计数
// 输入：
//   tp  时间点
// 输出：自 epoch 起的纳秒数
// 备注：跨进程共享同一内核时钟，订阅侧可直接比较该值
int64_t to_ns(std::chrono::steady_clock::time_point tp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

// 功能：按 "前缀/主题" 构造 iceoryx2 服务名
// 输入：
//   prefix  主题名前缀，非空
//   topic   主题名后缀，非空
// 输出：服务名对象
// 备注：名字非法（含非法字符或超长）时打印原因并直接退出进程，启动阶段无需恢复
iox2::ServiceName service_name(const std::string & prefix, const char * topic)
{
  auto exp = iox2::ServiceName::create((prefix + "/" + topic).c_str());
  if (!exp.has_value()) {
    logger()->error("[relay] 主题名非法：{}/{}", prefix, topic);
    std::exit(1);
  }
  return exp.value();
}

// 功能：从 iceoryx2 的初始化结果里取出对象
// 输入：
//   exp   iceoryx2 返回的 expected
//   what  失败环节的中文描述，用于日志
// 输出：expected 内部对象的引用
// 备注：只在启动阶段使用；没有 IPC 能力时 relay 无事可做，因此失败直接退出进程
template <typename Expected>
auto & unwrap_init(Expected & exp, const char * what)
{
  if (!exp.has_value()) {
    logger()->error("[relay] iceoryx2 初始化失败：{}", what);
    std::exit(1);
  }
  return exp.value();
}

// 功能：借一块共享内存样本填入载荷并发送
// 输入：
//   pub      该主题的发布者
//   payload  待发送的 POD 载荷
// 输出：无
// 备注：loan 失败（队列满）时静默丢弃本帧，转发链路不应因丢一帧阻塞
template <typename Publisher, typename Payload>
void publish(Publisher & pub, const char * channel, const Payload & payload)
{
  auto sample = pub.loan_uninit();
  if (!sample.has_value()) {
    comm_warnings().report(
      CommWarningKind::IpcLoanFailure, 0, 0, 0, channel);
    return;
  }
  auto sent = iox2::send(std::move(sample.value().write_payload(payload)));
  if (!sent.has_value()) {
    comm_warnings().report(
      CommWarningKind::IpcSendFailure, 0, 0, 0, channel);
  }
}
}  // namespace

int run(const RelayConfig & cfg, const std::function<bool()> & should_exit)
{
  using namespace iox2;

  // 先启动通信链路：init → open → read（读取线程由 start 拉起）
  auto io = io::make_io(cfg.io);
  if (!io) {
    logger()->error("[relay] 无法创建通信实现，link_type 配置有误");
    return 1;
  }
  CommBoard board(std::move(io), cfg);
  if (!board.init() || !board.open()) return 1;
  board.start();

  const std::string prefix = cfg.relay.prefix;
  const uint8_t link_kind =
    cfg.io.type == io::IoType::Serial ? static_cast<uint8_t>(msg::SourceKind::Serial)
                                      : static_cast<uint8_t>(msg::SourceKind::Can);

  auto node_exp = NodeBuilder().create<ServiceType::Ipc>();
  auto & node = unwrap_init(node_exp, "创建节点");

  // —— 发布侧：每个主题一个服务，载荷类型不同必须分开建 ——
  auto imu_name = service_name(prefix, msg::kTopicImu);
  auto imu_service_exp =
    node.service_builder(imu_name).publish_subscribe<msg::Imu>().open_or_create();
  auto & imu_service = unwrap_init(imu_service_exp, "创建 imu 服务");
  auto imu_pub_exp = imu_service.publisher_builder().create();
  auto & imu_pub = unwrap_init(imu_pub_exp, "创建 imu 发布者");

  auto ch_name =
    service_name(prefix, msg::kTopicChassis);
  auto ch_service_exp =
    node.service_builder(ch_name).publish_subscribe<msg::Chassis>().open_or_create();
  auto & ch_service = unwrap_init(ch_service_exp, "创建 chassis 服务");
  auto ch_pub_exp = ch_service.publisher_builder().create();
  auto & ch_pub = unwrap_init(ch_pub_exp, "创建 chassis 发布者");

  auto rb_name =
    service_name(prefix, msg::kTopicRobot);
  auto rb_service_exp =
    node.service_builder(rb_name).publish_subscribe<msg::Robot>().open_or_create();
  auto & rb_service = unwrap_init(rb_service_exp, "创建 robot 服务");
  auto rb_pub_exp = rb_service.publisher_builder().create();
  auto & rb_pub = unwrap_init(rb_pub_exp, "创建 robot 发布者");

  auto gm_name =
    service_name(prefix, msg::kTopicGame);
  auto gm_service_exp =
    node.service_builder(gm_name).publish_subscribe<msg::Game>().open_or_create();
  auto & gm_service = unwrap_init(gm_service_exp, "创建 game 服务");
  auto gm_pub_exp = gm_service.publisher_builder().create();
  auto & gm_pub = unwrap_init(gm_pub_exp, "创建 game 发布者");

  auto hl_name =
    service_name(prefix, msg::kTopicHealth);
  auto hl_service_exp =
    node.service_builder(hl_name).publish_subscribe<msg::Health>().open_or_create();
  auto & hl_service = unwrap_init(hl_service_exp, "创建 health 服务");
  auto hl_pub_exp = hl_service.publisher_builder().create();
  auto & hl_pub = unwrap_init(hl_pub_exp, "创建 health 发布者");

  // —— 订阅侧：外部进程下发的瞄准与速度指令 ——
  auto aim_name =
    service_name(prefix, msg::kTopicCmdAim);
  auto aim_service_exp =
    node.service_builder(aim_name).publish_subscribe<msg::AimCmd>().open_or_create();
  auto & aim_service = unwrap_init(aim_service_exp, "创建 cmd_aim 服务");
  auto aim_sub_exp = aim_service.subscriber_builder().create();
  auto & aim_sub = unwrap_init(aim_sub_exp, "创建 cmd_aim 订阅者");

  auto nav_name =
    service_name(prefix, msg::kTopicCmdNav);
  auto nav_service_exp =
    node.service_builder(nav_name).publish_subscribe<msg::NavCmd>().open_or_create();
  auto & nav_service = unwrap_init(nav_service_exp, "创建 cmd_nav 服务");
  auto nav_sub_exp = nav_service.subscriber_builder().create();
  auto & nav_sub = unwrap_init(nav_sub_exp, "创建 cmd_nav 订阅者");

  logger()->info("[relay] 主题前缀 {}，开始转发", prefix);

  // 节拍与各主题的发布周期
  constexpr auto kTick = std::chrono::microseconds(2000);          // 500Hz 主节拍
  constexpr auto kChassisPeriod = std::chrono::milliseconds(10);   // 100Hz
  constexpr auto kStatusPeriod = std::chrono::milliseconds(1000);  // 1Hz
  constexpr auto kHealthPeriod = std::chrono::milliseconds(100);   // 10Hz

  uint64_t seq_imu = 0, seq_ch = 0, seq_rb = 0, seq_gm = 0, seq_hl = 0;
  uint64_t tx_commands = 0;  // 已写串口的下行指令数
  int64_t last_rx_ns = 0;    // 最后一次收到上行帧的时刻
  uint64_t last_frame_count = board.rx_frame_count();  // 上一拍已处理的帧计数
  uint16_t last_latency_us = 0;  // 最近一帧 AimCmd 从视觉发出到写入串口的耗时

  auto next_tick = std::chrono::steady_clock::now();
  auto last_chassis = next_tick;
  auto last_status = next_tick;
  auto last_health = next_tick;

  while (!should_exit()) {
    const auto now = std::chrono::steady_clock::now();
    const int64_t now_ns = to_ns(now);

    // 周期性汇总输出通信告警，不在错误发生点直接打日志
    comm_warnings().flush_if_due();

    // ---- 下行：排空所有待处理指令，只执行最后一条 ----
    msg::AimCmd aim{};
    bool has_aim = false;
    auto taken = aim_sub.receive();
    if (!taken.has_value()) {
      comm_warnings().report(
        CommWarningKind::IpcReceiveFailure, 0, 0, 0, msg::kTopicCmdAim);
    }
    while (taken.has_value() && taken.value().has_value()) {
      aim = taken.value().value().payload();
      has_aim = true;
      taken = aim_sub.receive();
      if (!taken.has_value()) {
        comm_warnings().report(
          CommWarningKind::IpcReceiveFailure, 0, 0, 0, msg::kTopicCmdAim);
      }
    }
    if (has_aim) {
      info_relay::Command cmd{};
      cmd.shoot = aim.shoot != 0;
      cmd.yaw = aim.yaw_rad;
      cmd.pitch = aim.pitch_rad;
      board.send(cmd);
      tx_commands++;
      if (aim.t_ns > 0 && now_ns > static_cast<int64_t>(aim.t_ns)) {
        const int64_t us = (now_ns - static_cast<int64_t>(aim.t_ns)) / 1000;
        last_latency_us = static_cast<uint16_t>(us > 65535 ? 65535 : us);
      }
    }

    msg::NavCmd nav{};
    bool has_nav = false;
    auto nav_taken = nav_sub.receive();
    if (!nav_taken.has_value()) {
      comm_warnings().report(
        CommWarningKind::IpcReceiveFailure, 0, 0, 0, msg::kTopicCmdNav);
    }
    while (nav_taken.has_value() && nav_taken.value().has_value()) {
      nav = nav_taken.value().value().payload();
      has_nav = true;
      nav_taken = nav_sub.receive();
      if (!nav_taken.has_value()) {
        comm_warnings().report(
          CommWarningKind::IpcReceiveFailure, 0, 0, 0, msg::kTopicCmdNav);
      }
    }
    if (has_nav) {
      info_relay::NavCommand cmd{};
      cmd.vx = nav.vx;
      cmd.vy = nav.vy;
      cmd.wz = nav.wz;
      cmd.target_angle = 0.0;  // 线格式未携带目标航向角
      board.send(cmd);
      tx_commands++;
    }

    // ---- 上行：有新帧才发 Imu，避免重复样本 ----
    const uint64_t frames = board.rx_frame_count();
    if (frames != last_frame_count) {
      last_frame_count = frames;
      last_rx_ns = now_ns;

      const info_relay::ImuState imu = board.imu_state_at(now);
      msg::Imu m{};
      m.t_ns = static_cast<uint64_t>(now_ns);
      m.seq = ++seq_imu;
      m.uptime_ms = board.mcu_uptime_ms();
      m.gimbal_type = board.gimbal_type();
      m.gimbal_enable = board.gimbal_enabled() ? 1 : 0;
      m.autoaim_enabled = board.autoaim_enabled() ? 1 : 0;
      m.pitch_rad = static_cast<float>(imu.pitch);
      m.yaw_rad = static_cast<float>(imu.upper_yaw);  // IMU 装在小yaw 板上
      m.roll_rad = static_cast<float>(imu.roll);
      // 关节角 = 小yaw 世界角 - 大yaw 世界角；单yaw 构型下两者相等，恒为 0
      m.upper_yaw_rad = static_cast<float>(imu.upper_yaw - imu.lower_yaw);
      m.fold_yaw_rad = 0.0F;  // 折叠构型（gimbal_type=3）本赛制不使用
      publish(imu_pub, msg::kTopicImu, m);
    }

    // ---- 底盘反馈：固定 100Hz 发布最新快照 ----
    if (now - last_chassis >= kChassisPeriod) {
      last_chassis = now;
      const auto ch = board.chassis_feedback();
      msg::Chassis c{};
      c.t_ns = static_cast<uint64_t>(now_ns);
      c.seq = ++seq_ch;
      c.chassis_enabled = ch.enabled ? 1 : 0;
      c.speed_x = ch.speed_x;
      c.speed_y = ch.speed_y;
      c.speed_turn = ch.speed_turn;
      c.feedback_angle = ch.feedback_angle;
      publish(ch_pub, msg::kTopicChassis, c);
    }

    // ---- 机器人与比赛状态：1Hz ----
    if (now - last_status >= kStatusPeriod) {
      last_status = now;

      const auto rs = board.referee_state();
      const auto own_red = board.own_team_red();

      msg::Robot r{};
      r.t_ns = static_cast<uint64_t>(now_ns);
      r.seq = ++seq_rb;
      r.robot_id = board.robot_id();
      r.own_team_red = own_red.has_value() ? (own_red.value() ? 1 : 0) : 2;
      r.current_hp = static_cast<uint16_t>(rs.current_hp < 0 ? 0 : rs.current_hp);
      r.maximum_hp = static_cast<uint16_t>(rs.maximum_hp < 0 ? 0 : rs.maximum_hp);
      r.barrel_heat_17mm = board.barrel_heat();
      r.heat_limit = board.heat_limit();
      // bit0 机器人状态已收到、bit1 比赛状态已收到、bit2 保留（恒为 1 表示 relay 在线）
      r.valid_mask = static_cast<uint8_t>(
        (rs.robot_status_valid ? 0x01 : 0x00) | (rs.game_status_valid ? 0x02 : 0x00) | 0x04);
      publish(rb_pub, msg::kTopicRobot, r);

      const auto gs = board.game_status();
      msg::Game g{};
      g.t_ns = static_cast<uint64_t>(now_ns);
      g.seq = ++seq_gm;
      g.game_type = gs.game_type;
      g.game_progress = gs.game_progress;
      g.in_battle = board.is_game_started() ? 1 : 0;
      g.valid = rs.game_status_valid ? 1 : 0;
      publish(gm_pub, msg::kTopicGame, g);
    }

    // ---- 链路健康：10Hz，链路失效时照常发布，由字段表达状态 ----
    if (now - last_health >= kHealthPeriod) {
      last_health = now;

      const bool link_ok =
        last_rx_ns != 0 && (now_ns - last_rx_ns) < static_cast<int64_t>(cfg.relay.timeout_ms) * 1000000;

      msg::Health h{};
      h.t_ns = static_cast<uint64_t>(now_ns);
      h.seq = ++seq_hl;
      h.last_rx_ns = static_cast<uint64_t>(last_rx_ns < 0 ? 0 : last_rx_ns);
      h.rx_frames = static_cast<uint32_t>(frames);
      h.crc_errors = static_cast<uint32_t>(
        comm_warnings().count(CommWarningKind::Crc8Failure) +
        comm_warnings().count(CommWarningKind::Crc16Failure));
      h.tx_commands = static_cast<uint32_t>(tx_commands);
      h.transport_ok = link_ok ? 1 : 0;
      h.source_kind = link_kind;
      h.last_latency_us = last_latency_us;
      h.schema_version = msg::kSchemaVersion;  // 运行期握手：视觉侧据此判断契约是否同版
      publish(hl_pub, msg::kTopicHealth, h);
    }

    next_tick += kTick;
    std::this_thread::sleep_until(next_tick);
  }

  comm_warnings().flush();
  logger()->info("[relay] 退出：下行 {} 条，上行 {} 帧", tx_commands, last_frame_count);
  return 0;
}

}  // namespace info_relay
