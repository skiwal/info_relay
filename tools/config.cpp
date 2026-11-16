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
 * 配置加载实现：逐段读取 YAML，并把路径字段解析成绝对路径。
 */
#include "tools/config.hpp"

#include <stdexcept>

#include "tools/paths.hpp"
#include "tools/yaml.hpp"
#include "msg/relay_msgs.hpp"

namespace info_relay
{
namespace
{

io::IoType parse_io_type(const std::string & text)
{
  if (text.empty() || text == "serial") return io::IoType::Serial;
  if (text == "can") return io::IoType::Can;
  throw std::runtime_error("未知的 link_type: " + text);
}

io::IoOptions load_io(const YAML::Node & yaml)
{
  io::IoOptions options;
  options.type = parse_io_type(value<std::string>(yaml, "link_type", "serial"));

  if (options.type == io::IoType::Serial) {
    options.device = require<std::string>(yaml, "com_port");
    options.baud_rate = value<uint32_t>(yaml, "baud_rate", options.baud_rate);
  } else {
    options.device = require<std::string>(yaml, "can_interface");
    options.can_id = static_cast<uint32_t>(value<int>(yaml, "can_id", 0x123));
  }
  options.timeout_ms = value<int>(yaml, "io_timeout_ms", options.timeout_ms);
  return options;
}

BoardSection load_board(const YAML::Node & yaml)
{
  BoardSection board;
  board.bullet_speed = value<double>(yaml, "bullet_speed", board.bullet_speed);
  board.use_referee_system = value<bool>(yaml, "use_referee_system", board.use_referee_system);
  board.game_start_progress_threshold =
    value<int>(yaml, "game_start_progress_threshold", board.game_start_progress_threshold);
  board.heat_stop_margin = value<int>(yaml, "heat_stop_margin", board.heat_stop_margin);
  board.imu_on_upper_axis = value<std::string>(yaml, "gimbal_imu_axis", "upper") != "lower";
  return board;
}

}  // namespace

RelayConfig RelayConfig::load(const std::string & path) { return load(info_relay::load(path)); }

RelayConfig RelayConfig::load(const YAML::Node & yaml)
{
  RelayConfig cfg;
  cfg.io = load_io(yaml);
  cfg.board = load_board(yaml);

  cfg.relay.prefix = value<std::string>(yaml, "relay_prefix", std::string(msg::kDefaultPrefix));
  cfg.relay.timeout_ms = value<int>(yaml, "relay_timeout_ms", cfg.relay.timeout_ms);

  const auto log_dir = value<std::string>(yaml, "log_dir", "");
  cfg.log.dir = log_dir.empty() ? std::string() : paths::resolve(log_dir).string();
  return cfg;
}

}  // namespace info_relay
