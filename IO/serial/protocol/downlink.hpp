/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__SERIAL__PROTOCOL__DOWNLINK_HPP
#define INFO_RELAY__IO__SERIAL__PROTOCOL__DOWNLINK_HPP

#include <cstddef>
#include <cstdint>

// ============================================================================
// 上位机 → 下位机 的下行帧：
//   0x0400  自瞄控制（pitch/yaw/shoot）
//   0x0401  导航速度（vx/vy/wz/target_angle）
//
// 接收方向（下位机 → 上位机）按下位机固件的协议分层解析：
//   · 线格式与分帧    IO/serial/protocol/frame.hpp
//   · host 上行 0x05xx IO/serial/protocol/uplink_host.hpp
//   · 裁判转发 0x0xxx  IO/serial/protocol/uplink_referee.hpp
//   · 号段分派        IO/serial/protocol/dispatch.hpp
//
// 下行帧 ID、长度和字段顺序必须与下位机固件同步；固件按 sizeof 做严格长度校验。
// ============================================================================

namespace info_relay
{

// 下行帧命令 ID：0x0400 自瞄 / 0x0401 导航
inline constexpr uint16_t kCmdAimDownlink = 0x0400;
inline constexpr uint16_t kCmdNavDownlink = 0x0401;

// 裁判系统帧头（5 字节，cmd_id 紧随其后）
struct Header
{
  uint8_t header = 0xA5;
  uint16_t data_length = 15;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
  uint16_t cmd_id;
} __attribute__((packed));

// cmd_id 0x0400 — vision → MCU aim command（data 段 9 字节）
struct SendPacket
{
  Header header;
  float pitch = 0.0f;  // 世界系绝对 pitch（rad）
  float yaw = 0.0f;    // 世界系绝对 yaw（rad）
  uint8_t shoot = 0;   // 1 = 允许发弹
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x0401 — vision → MCU navigation command（data 段 16 字节）
struct NavigationPacket
{
  Header header;
  float vx;            // 体坐标系前进速度，m/s
  float vy;            // 体坐标系左移速度，m/s；★打包前由调用方取反
  float wz;            // 转向角速度，rad/s
  float target_angle;  // 目标航向角，rad；当前固定发 0
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

inline constexpr size_t kAimDataSize = sizeof(SendPacket) - sizeof(Header) - 2;
inline constexpr size_t kNavDataSize = sizeof(NavigationPacket) - sizeof(Header) - 2;

struct AimFields
{
  float pitch = 0.0f;
  float yaw = 0.0f;
  uint8_t shoot = 0;
};

struct NavFields
{
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  float target_angle = 0.0f;
};

// 完整帧打包：成功返回写入字节数，失败返回 0
size_t encode_aim(
  uint8_t * out, size_t capacity, float pitch, float yaw, uint8_t shoot, uint8_t seq = 0);
size_t encode_nav(
  uint8_t * out, size_t capacity, float vx, float vy, float wz, float target_angle,
  uint8_t seq = 0);

// 完整帧解包：校验 ID、长度与 CRC；成功返回 true
bool decode_aim(const uint8_t * data, size_t size, AimFields & out);
bool decode_nav(const uint8_t * data, size_t size, NavFields & out);

// data 段解包：供已由分帧器校验过 ID/长度/CRC 的接收路径使用
bool decode_aim_payload(const uint8_t * data, size_t size, AimFields & out);
bool decode_nav_payload(const uint8_t * data, size_t size, NavFields & out);

// 下行帧长必须与固件结构体完全一致（固件按 sizeof 严格校验，不一致会整帧丢弃）
static_assert(
  kAimDataSize == 9,
  "0x0400 data 段必须是 9 字节（pitch/yaw/shoot），改动需同步下位机固件");
static_assert(
  kNavDataSize == 16,
  "0x0401 data 段必须是 16 字节（vx/vy/wz/target_angle），改动需同步下位机固件");

// 解析后的比赛状态（裁判 0x0001，由 CommBoard 从 v2027::game_status_t 搬运而来）
struct GameStatus
{
  uint8_t game_type;          // 比赛类型
  uint8_t game_progress;      // 比赛阶段 (4 = 比赛中)
  uint16_t stage_remain_time; // 阶段剩余时间
  uint64_t sync_timestamp;    // 同步时间戳
};

// 解析后的机器人状态（裁判 0x0201，由 CommBoard 从 v2027::robot_status_t 搬运而来）
struct RobotStatus
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_hp;
  uint16_t max_hp;
  uint16_t shooter_cooling_rate;
  uint16_t shooter_heat_limit;
  uint16_t chassis_power_limit;
  float bullet_speed_limit;  // 2027 新增：射击初速度上限 m/s
  bool mains_power_gimbal;   // 云台供电
  bool mains_power_chassis;  // 底盘供电
  bool mains_power_shooter;  // 发射机构供电
};

}  // namespace info_relay

#endif  // INFO_RELAY__IO__SERIAL__PROTOCOL__DOWNLINK_HPP
