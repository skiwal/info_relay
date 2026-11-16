/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__CONFIG_HPP
#define INFO_RELAY__CONFIG_HPP

/*
 * 中转进程的配置：串口 / 录制 / 跨进程主题 / 日志 / 异常注入五段。
 *
 * 数据流：configs/info_relay.yaml → RelayConfig::load() → 各子系统按段取值。
 *
 * 依赖与边界：只描述本进程自己的参数，**不含相机段**（相机属于视觉进程）；
 *             路径类字段在加载时统一解析成绝对路径，运行期不再拼接。
 *
 * 使用注意：加载失败抛 std::runtime_error；缺必需键（com_port）同样抛异常，
 *           避免带着错误参数静默启动。
 */

#include <cstdint>
#include <string>

#include <yaml-cpp/yaml.h>

#include "common/fault.hpp"
#include "common/source_mode.hpp"

namespace info_relay
{

// 串口与数据源参数
struct SerialSection
{
  SourceMode mode = SourceMode::Real;  // 数据源：真实 / 虚拟 / 回放
  std::string port = "/dev/ttyACM0";   // 串口设备路径
  uint32_t baud = 921600;              // 波特率，必须与下位机一致
  bool external = false;               // fake 模式是否使用外部 pty
  double imu_hz = 500.0;               // 虚拟云台姿态频率，Hz
  double chassis_hz = 50.0;            // 虚拟底盘反馈频率，Hz
  double status_hz = 1.0;              // 虚拟裁判转发频率，Hz
  double reset_period_s = 0.0;         // 虚拟 MCU 重启周期，s；0 表示不注入
  std::string replay_file;             // 回放文件，加载时解析为绝对路径
  uint8_t robot_id = 0;                // 虚拟机器人 ID，0 表示不模拟裁判身份
  uint16_t heat = 30;                  // 虚拟上报的 17mm 热量
  uint16_t fake_hp = 400;              // 虚拟上报的当前血量
  uint16_t fake_max_hp = 400;          // 虚拟上报的血量上限
};

// 串口原始字节录制
struct RecordSection
{
  bool enable = false;  // 是否录制 serial.bin
  std::string dir;      // 会话目录，加载时解析为绝对路径
};

// 跨进程通信
struct RelaySection
{
  std::string prefix = "rm_vision";  // 主题前缀，必须与视觉侧一致
  int timeout_ms = 200;              // 无上行帧判定链路失效的阈值，ms
};

// 日志
struct LogSection
{
  std::string dir;  // 日志目录；空串表示使用 <home>/logs
};

// 完整配置
struct RelayConfig
{
  SourceMode source_mode = SourceMode::Real;  // 总开关，serial.mode 可单独覆盖
  int seed = 20261015;                        // 故障注入随机种子，固定以便复现
  SerialSection serial;                       // 串口段
  RecordSection record;                       // 录制段
  RelaySection relay;                         // 跨进程段
  LogSection log;                             // 日志段
  FaultConfig fault;                          // 异常注入段

  // 功能：从配置文件加载全部参数
  // 输入：
  //   path  YAML 文件路径
  // 输出：填好的配置；路径字段已解析为绝对路径
  // 备注：文件不存在、语法错误或缺少 com_port 时抛 std::runtime_error
  static RelayConfig load(const std::string & path);

  // 功能：从已解析的 YAML 节点加载
  // 输入：
  //   yaml  根节点
  // 输出：填好的配置
  // 备注：供测试直接喂节点，避免落文件
  static RelayConfig load(const YAML::Node & yaml);
};

}  // namespace info_relay

#endif  // INFO_RELAY__CONFIG_HPP
