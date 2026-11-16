/*
 * 配置加载的实现：逐段读取 YAML，并把所有路径字段解析成绝对路径。
 *
 * 注意：契约见 config.hpp，本文件只写实现要点。
 *       路径在这里一次性解析完，运行期各子系统直接拿到可直接使用的绝对路径，
 *       避免"相对谁"的歧义在不同工作目录下表现不一致。
 */
#include "config.hpp"

#include "common/paths.hpp"
#include "common/yaml.hpp"
#include "msg/relay_msgs.hpp"

namespace info_relay
{
namespace
{
// 功能：读取串口段
// 输入：
//   yaml      根节点
//   fallback  总开关给出的默认模式
// 输出：串口段配置
// 备注：com_port 为必需项；缺失直接抛异常
SerialSection load_serial(const YAML::Node & yaml, SourceMode fallback)
{
  SerialSection s;
  s.mode = parse_source_mode(value<std::string>(yaml, "serial_mode", ""), fallback);
  s.port = require<std::string>(yaml, "com_port");
  s.baud = value<uint32_t>(yaml, "baud_rate", s.baud);
  s.external = value<bool>(yaml, "fake_serial_external", s.external);
  s.imu_hz = value<double>(yaml, "fake_imu_hz", s.imu_hz);
  s.chassis_hz = value<double>(yaml, "fake_chassis_hz", s.chassis_hz);
  s.status_hz = value<double>(yaml, "fake_status_hz", s.status_hz);
  s.reset_period_s = value<double>(yaml, "fake_reset_period_s", s.reset_period_s);
  s.robot_id = static_cast<uint8_t>(value<int>(yaml, "fake_robot_id", s.robot_id));
  s.heat = static_cast<uint16_t>(value<int>(yaml, "fake_heat", s.heat));
  s.fake_hp = static_cast<uint16_t>(value<int>(yaml, "fake_hp", s.fake_hp));
  s.fake_max_hp = static_cast<uint16_t>(value<int>(yaml, "fake_max_hp", s.fake_max_hp));
  s.replay_file =
    paths::resolve(value<std::string>(yaml, "serial_replay_file", "records/relay_session/serial.bin"))
      .string();
  return s;
}

// 功能：读取录制段
// 输入：
//   yaml  根节点
// 输出：录制段配置，dir 已解析为绝对路径
// 备注：没有 record: 段时保持默认（不录制）
RecordSection load_record(const YAML::Node & yaml)
{
  RecordSection r;
  const auto node = yaml["record"];
  r.enable = value<bool>(node, "enable", r.enable);
  r.dir = paths::resolve(value<std::string>(node, "dir", "records/relay_session")).string();
  return r;
}
}  // namespace

RelayConfig RelayConfig::load(const std::string & path) { return load(info_relay::load(path)); }

RelayConfig RelayConfig::load(const YAML::Node & yaml)
{
  RelayConfig cfg;
  cfg.source_mode = parse_source_mode(value<std::string>(yaml, "source_mode", "real"), SourceMode::Real);
  cfg.seed = value<int>(yaml, "fault_seed", cfg.seed);

  cfg.serial = load_serial(yaml, cfg.source_mode);
  cfg.record = load_record(yaml);
  cfg.fault = FaultConfig::load(yaml);

  cfg.relay.prefix = value<std::string>(yaml, "relay_prefix", std::string(msg::kDefaultPrefix));
  cfg.relay.timeout_ms = value<int>(yaml, "relay_timeout_ms", cfg.relay.timeout_ms);

  // 日志目录：留空表示由调用方回落到 <home>/logs；非空时按仓库根解析相对路径。
  // 注意不能直接对空串调用 resolve()——那会返回 home()，导致日志写到仓库根目录。
  const auto log_dir = value<std::string>(yaml, "log_dir", "");
  cfg.log.dir = log_dir.empty() ? std::string() : paths::resolve(log_dir).string();

  return cfg;
}

}  // namespace info_relay
