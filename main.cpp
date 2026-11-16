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
 * info_relay 进程入口：读配置 → 初始化日志 → 进入 src 下的 relay 主流程。
 *
 * 通信链路的 init/open/read 由 relay 按配置驱动；本文件不直接依赖串口或 CAN 实现。
 * 退出信号由 Exiter 统一收敛，relay 每拍检查一次。
 */
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>

#include "src/relay.hpp"
#include "tools/config.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/paths.hpp"

int main(int argc, char * argv[])
{
  const std::string config_path =
    argc >= 2 ? std::string(argv[1]) : info_relay::paths::default_config().string();

  info_relay::RelayConfig cfg;
  try {
    cfg = info_relay::RelayConfig::load(config_path);
    info_relay::init_logging(
      cfg.log.dir.empty() ? info_relay::paths::logs() : std::filesystem::path(cfg.log.dir));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[info_relay] 启动失败：%s\n", e.what());
    return 1;
  }

  info_relay::logger()->info(
    "[info_relay] 配置 {}，仓库根 {}", config_path, info_relay::paths::home().string());

  info_relay::Exiter exiter;
  return info_relay::run(cfg, [&exiter] { return exiter.exit(); });
}
