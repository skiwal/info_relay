/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__COMMON__PATHS_HPP
#define INFO_RELAY__COMMON__PATHS_HPP

/*
 * 运行期路径解析：所有目录都由「仓库根」派生，保证产物不写到仓库之外。
 *
 * 数据流：main() 启动时先调用 home()，其余路径函数都以它为基础；
 *         日志目录与会话录制目录可被配置覆盖，但默认值仍落在仓库内。
 *
 * 依赖与边界：只依赖 <filesystem> 与 POSIX 的 /proc/self/exe，不依赖第三方库；
 *             解析失败时退化为当前工作目录并打印一次 warn，绝不静默写错位置。
 *
 * 使用注意：home() 的结果在进程生命周期内视为常量，chdir 不会影响它；
 *           相对路径一律相对 home() 解析，而不是相对当前工作目录。
 */

#include <filesystem>
#include <string>

namespace info_relay::paths
{

// 功能：解析仓库根目录
// 输入：无
// 输出：仓库根路径；优先级 INFO_RELAY_HOME > 可执行文件所在目录（在 build/ 下则取父目录）> 当前工作目录
// 备注：结果缓存，重复调用不再读环境变量；退化路径只打印一次 warn
std::filesystem::path home();

// 功能：取默认日志目录
// 输入：无
// 输出：<home>/logs
// 备注：目录本身不在这里创建，由 init_logging() 负责
std::filesystem::path logs();

// 功能：取串口录制根目录
// 输入：无
// 输出：<home>/records
// 备注：具体会话子目录由配置 record.dir 决定
std::filesystem::path records();

// 功能：取默认配置文件路径
// 输入：无
// 输出：<home>/configs/info_relay.yaml
// 备注：仅在命令行未给参数时使用
std::filesystem::path default_config();

// 功能：把配置里的路径解析为绝对路径
// 输入：
//   p  配置值，允许相对路径与空串
// 输出：绝对路径；p 为空时返回 home()
// 备注：相对路径相对 home() 解析，因此从任意工作目录启动都指向同一位置
std::filesystem::path resolve(const std::string & p);

}  // namespace info_relay::paths

#endif  // INFO_RELAY__COMMON__PATHS_HPP
