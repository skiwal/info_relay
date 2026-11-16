/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__TYPES_HPP
#define INFO_RELAY__TYPES_HPP

/*
 * 中转进程内部使用的业务结构体：下行指令、导航指令、裁判状态、云台姿态。
 *
 * 数据流：解析结果 → ImuState / RefereeState → 打包成 msg 里的 POD 发布；
 *         视觉/导航下发的 AimCmd/NavCmd → Command/NavCommand → 打包成 0x0402/0x0405 写串口。
 *
 * 依赖与边界：只放纯数据，不含任何算法；本文件不依赖 third_party 与视觉工程。
 *
 * 注意：这里的结构体是**进程内**表示，允许用 double 与 bool；
 *       跨进程线格式见 msg/relay_msgs.hpp，两者不是同一份东西，转换处必须显式逐字段赋值。
 */

#include <cstdint>

namespace info_relay
{

// 视觉下发的云台瞄准指令（对应下行 0x0402）
struct Command
{
  bool control = false;          // 是否接管云台；false 时下位机忽略 yaw/pitch
  bool shoot = false;            // 是否请求发弹
  double yaw = 0;                // 世界系绝对 yaw，rad；(0, 2π]
  double pitch = 0;              // 世界系绝对 pitch，rad，向上为负
  double horizon_distance = 0;   // 弹道补偿用的水平距离，m；为 0 时下位机忽略
};

// 导航速度指令（对应下行 0x0405）
struct NavCommand
{
  double vx = 0;            // 体坐标系前进速度，m/s
  double vy = 0;            // 体坐标系左移速度，m/s；写报文时会取反
  double wz = 0;            // 转向角速度，rad/s，逆时针为正
  double target_angle = 0;  // 目标航向角，rad；导航侧当前不发，恒为 0
};

// 裁判状态快照
struct RefereeState
{
  uint8_t game_progress = 0;   // 0 未开始 / 1 准备 / 2 自检 / 3 倒计时 / 4 比赛中 / 5 结算
  int32_t current_hp = 0;      // 当前血量
  int32_t maximum_hp = 0;      // 血量上限
  bool game_status_valid = false;   // 是否收到过 0x0001
  bool robot_status_valid = false;  // 是否收到过 0x0201
};

// 一帧云台姿态（弧度），全部是「上电时车头朝向为 0」的世界系绝对角：
//   大yaw(lower) 只有 yaw；pitch/roll 属于小yaw(upper)，即主相机所在的那块板。
struct ImuState
{
  double pitch = 0.0;      // 小yaw 板俯仰，低头为正
  double roll = 0.0;       // 小yaw 板横滚
  double upper_yaw = 0.0;  // 小yaw 世界系绝对 yaw，[0, 2π)
  double lower_yaw = 0.0;  // 大yaw 世界系绝对 yaw，[0, 2π)
};

}  // namespace info_relay

#endif  // INFO_RELAY__TYPES_HPP
