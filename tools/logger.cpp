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
 * 日志门面的实现：spdlog 控制台 + 文件双 sink，文件名带启动时刻。
 *
 * 数据流：init_logging(dir) → create_directories(dir) → basic_file_sink(dir/<时间>.log)
 *         → logger() 返回该 logger；未初始化时惰性回落到 paths::logs()。
 *
 * 注意：契约见 logger.hpp，本文件只写实现要点。
 */
#include "tools/logger.hpp"

#include <fmt/chrono.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <chrono>
#include <filesystem>
#include <system_error>
#include <vector>

#include "tools/paths.hpp"

namespace info_relay
{
namespace
{
std::shared_ptr<spdlog::logger> global_logger;  // 全局 logger，惰性创建后常驻

// 功能：按目录建立 logger（控制台 + 文件）
// 输入：
//   dir  日志目录
// 输出：新建的 logger，永不为空
// 备注：目录创建失败或文件打不开时静默退化为仅控制台，不影响业务
std::shared_ptr<spdlog::logger> build_logger(const std::filesystem::path & dir)
{
  std::vector<spdlog::sink_ptr> sinks;

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);
  sinks.push_back(console_sink);

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (!ec) {
    try {
      const auto file_name =
        dir / fmt::format("{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name.string(), true);
      file_sink->set_level(spdlog::level::debug);
      sinks.push_back(file_sink);
    } catch (const spdlog::spdlog_ex &) {
      // 文件 sink 打不开时只保留控制台输出
    }
  }

  auto result = std::make_shared<spdlog::logger>("", sinks.begin(), sinks.end());
  result->set_level(spdlog::level::debug);
  result->flush_on(spdlog::level::info);
  return result;
}
}  // namespace

void init_logging(const std::filesystem::path & dir) { global_logger = build_logger(dir); }

std::shared_ptr<spdlog::logger> logger()
{
  if (!global_logger) global_logger = build_logger(paths::logs());
  return global_logger;
}

}  // namespace info_relay
