#ifndef INFO_RELAY__COMMON__LOGGER_HPP
#define INFO_RELAY__COMMON__LOGGER_HPP

/*
 * 日志门面：控制台 + 文件双输出，文件目录由调用方指定。
 *
 * 数据流：init_logging(dir) 建立 sink → 各处调用 logger()->info/warn/error。
 *
 * 依赖与边界：直接用 spdlog，不经过任何其他工程的日志封装；
 *             文件 sink 打不开时只保留控制台输出，不把日志初始化变成启动失败点。
 *
 * 使用注意：init_logging() 必须在第一次 logger() 之前调用——串口链路构造期就会写日志，
 *           所以 main() 里读配置后要立刻初始化；未初始化时 logger() 会回落到
 *           paths::logs()，仍保证写在仓库目录内。
 */

#include <spdlog/spdlog.h>

#include <filesystem>
#include <memory>

namespace info_relay
{

// 功能：建立日志输出，创建目录并打开当次运行的文件 sink
// 输入：
//   dir  日志目录，绝对路径；不存在时自动创建
// 输出：无
// 备注：重复调用会重建 logger（旧句柄仍被持有者使用，通常只在启动阶段调用一次）
void init_logging(const std::filesystem::path & dir);

// 功能：取全局 logger
// 输入：无
// 输出：spdlog logger 共享指针，永不为空
// 备注：首次调用惰性创建；未显式初始化时文件目录取 paths::logs()
std::shared_ptr<spdlog::logger> logger();

}  // namespace info_relay

#endif  // INFO_RELAY__COMMON__LOGGER_HPP
