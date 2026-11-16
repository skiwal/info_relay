/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef IO__SERIAL_BOARD_HPP
#define IO__SERIAL_BOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "types.hpp"
#include "link/protocol/gimbal/packet.hpp"
#include "link/protocol/link.hpp"
#include "link/protocol/referee.hpp"
#include "config.hpp"
#include "link/transport/serial_recorder.hpp"
#include "link/transport/transport.hpp"
#include "common/queue.hpp"

namespace info_relay
{

// ImuState 已提升到 io/command.hpp，与视觉侧的跨进程载荷共用同一份定义
class SerialBoard
{
public:
  double bullet_speed;

  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();

  // 一帧 IMU 姿态（按图像时间戳插值）。双yaw 平台的各坐标系由调用方用 tf 树复合。
  ImuState imu_state_at(std::chrono::steady_clock::time_point timestamp);

  // 兼容接口（单yaw 兵种）：Rz(lower_yaw)·Ry(pitch)·Rx(roll)。
  // 双yaw 平台请改用 imu_state_at()，因为大yaw 只有 yaw、pitch/roll 属于小yaw。
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;
  void send(NavCommand command) const;

  // Game / match state（来自下位机转发的裁判 0x0001 / 0x0201）
  GameStatus game_status() const;
  RobotStatus robot_status() const;
  RefereeState referee_state() const;
  bool is_game_started() const;

  // 自己的机器人 ID（裁判 0x0201 的 robot_id，uint8_t）。
  // 红方 1~11，蓝方 101~111（蓝方 = 红方 + 100），2/102 保留位；0 表示还没收到。
  uint8_t robot_id() const;

  // 己方是否红方。红方 1~11、蓝方 101~111，ID 非法/未收到时返回 nullopt，
  // 调用方应回退到配置里的 enemy_color。
  std::optional<bool> own_team_red() const;

  // 枪管热量：当前值来自裁判 0x0202 的 shooter_17mm_barrel_heat，
  // 上限来自 0x0201 的 shooter_barrel_heat_limit。
  uint16_t barrel_heat() const;
  uint16_t heat_limit() const;

  // 是否允许开火：热量接近上限时提前停火（提前量 heat_stop_margin），
  // 回落到 limit - 2*margin 后自动恢复；没有裁判数据（limit 为 0）时恒为 true。
  bool can_shoot() const;

  // MCU 重启信号（0x0500 的 uptime_ms 回落，等价旧 0x505 的 reset 位）
  bool reset_pending();

  // 底盘反馈快照（源 0x0502），供 info_relay 转发
  struct ChassisFeedback
  {
    bool enabled = false;      // 底盘使能，0/1
    float speed_x = 0;         // 实测前进速度，m/s，体坐标（前为正）
    float speed_y = 0;         // 实测左移速度，m/s，体坐标（左为正）
    float speed_turn = 0;      // 实测转向角速度，rad/s，逆时针为正
    float feedback_angle = 0;  // 底盘航向角，rad，世界系；不是 IMU 的 yaw
  };

  // 功能：取底盘反馈快照
  // 输入：无
  // 输出：ChassisFeedback；从未收到 0x0502 时各字段为默认值且 enabled=false
  // 备注：加 status_mutex_，可在 relay 主循环线程调用
  ChassisFeedback chassis_feedback() const;

  // 返回值：已成功解码的上行帧累计数（任意 cmd_id）
  // 用途：relay 用它判断"有没有新样本"，避免按固定频率重复发布同一帧
  uint64_t rx_frame_count() const { return rx_frame_count_.load(); }

  // 返回值：MCU 上电时间，ms（源 0x0500）；0 表示尚未收到握手帧
  uint32_t mcu_uptime_ms() const { return uptime_ms_.load(); }

  // 返回值：云台构型（源 0x0500）：0 无 / 1 gimbal / 2 双yaw / 3 折叠
  uint8_t gimbal_type() const { return gimbal_type_.load(); }

  // 返回值：云台软件使能（源 0x0503），false 时电机不驱动但角度仍更新
  bool gimbal_enabled() const { return gimbal_enabled_.load(); }

  // 返回值：MCU 对自瞄的最终裁决（源 0x0503），上位机只服从不自裁
  bool autoaim_enabled() const { return autoaim_enabled_.load(); }

private:
  struct IMUData
  {
    ImuState state;  // 四个原始角度，按时间戳插值后再复合出各坐标系姿态
    std::chrono::steady_clock::time_point timestamp;
  };

  // 把队列里的新样本搬进 data_ahead_ / data_behind_
  void update_imu_samples();

  // 解码器每解出一帧回调一次（接收线程内），把载荷搬进本类的业务状态
  void on_frame(uint16_t cmd_id);
  void handle_gimbal(const host_comm::host_gimbal_status_t & pkt);
  void handle_game_status(const referee_comm::v2027::game_status_t & pkt);
  void handle_robot_status(const referee_comm::v2027::robot_status_t & pkt);
  void handle_power_heat(const referee_comm::v2027::power_heat_data_t & pkt);
  // 热量门限状态机：接近上限锁定开火，回落后解锁（仅在接收线程内调用）
  void update_heat_gate();

  // 数据源：真实串口 / 内置虚拟下位机 / 原始字节回放，由配置决定，解析逻辑完全一致
  sim::IByteTransport::Ptr transport_;
  SourceMode source_mode_ = SourceMode::Real;
  std::shared_ptr<sim::SerialRecorder> recorder_;

  std::string device_name_;
  uint32_t baud_rate_ = 921600;

  // 链路解包：分帧器 + 号段分派（0x05xx 上行 / 0x0001~0x0A06 裁判转发）
  MinipcLink link_;
  referee_comm::Receiver receiver_{link_};

  std::thread receive_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex send_mutex_;
  mutable std::mutex status_mutex_;

  Queue<IMUData> queue_{1000};
  IMUData data_ahead_;
  IMUData data_behind_;

  // Robot state from MCU
  GameStatus game_status_{};
  RobotStatus robot_status_{};
  std::atomic<bool> game_started_{false};
  std::atomic<bool> reset_pending_{false};
  std::atomic<bool> game_status_received_{false};
  std::atomic<bool> robot_status_received_{false};
  bool use_referee_system_ = false;
  bool test_nav_ = false;
  RefereeState test_nav_state_{};
  int game_start_progress_threshold_ = 4;

  // 裁判 0x0201 / 0x0202 的机器人身份与枪管热量（接收线程写，业务线程读 status_mutex_）
  uint8_t robot_id_ = 0;  // 红方 1~11，蓝方 101~111；0 = 未知
  uint16_t barrel_heat_ = 0;
  uint16_t heat_limit_ = 0;
  int heat_stop_margin_ = 10;           // 停火提前量（热量单位），配置项 heat_stop_margin
  std::atomic<bool> heat_blocked_{false};  // true = 热量过高，暂停开火

  // 0x0503 的 yaw_angle 是 IMU（相机/枪管）世界系 yaw。双yaw 平台上 IMU 装在小yaw 板，
  // 因此 小yaw 世界角 = yaw_angle，大yaw 世界角 = yaw_angle - upper_yaw_angle。
  // 若实车把 IMU 装在大yaw 轴上，把 gimbal_imu_axis 配成 lower 即可互换。
  bool imu_on_upper_axis_ = true;

  // 对外观测量：接收线程写、relay 主循环读，用原子量避免加锁
  std::atomic<uint64_t> rx_frame_count_{0};  // 已解码上行帧累计数
  std::atomic<uint32_t> uptime_ms_{0};       // 最近一次 0x0500 的上电时间，ms
  std::atomic<uint8_t> gimbal_type_{0};      // 最近一次 0x0500 的云台构型
  std::atomic<bool> gimbal_enabled_{false};    // 最近一次 0x0503 的云台软件使能
  std::atomic<bool> autoaim_enabled_{false};   // 最近一次 0x0503 的自瞄裁决
  ChassisFeedback chassis_feedback_{};       // 底盘快照；接收线程与读方都在 status_mutex_ 内访问

  uint32_t last_uptime_ms_ = 0;
  bool config_logged_ = false;
  bool first_imu_received_ = false;

  void receiveThread();
  void reconnect();
};

}  // namespace info_relay

#endif  // IO__SERIAL_BOARD_HPP
