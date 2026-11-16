/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/serial/protocol/downlink.hpp"

#include <cstring>

#include "IO/serial/protocol/crc.hpp"

namespace info_relay
{
namespace
{

template <typename Packet>
size_t encode_packet(uint8_t * out, size_t capacity, const Packet & packet)
{
  if (out == nullptr || capacity < sizeof(Packet)) return 0;
  std::memcpy(out, &packet, sizeof(Packet));
  return sizeof(Packet);
}

}  // namespace

size_t encode_aim(
  uint8_t * out, size_t capacity, float pitch, float yaw, uint8_t shoot, uint8_t seq)
{
  SendPacket packet{};
  packet.header.header = 0xA5;
  packet.header.data_length = static_cast<uint16_t>(kAimDataSize);
  packet.header.seq = seq;
  packet.header.cmd_id = kCmdAimDownlink;
  crc8::Append_CRC8_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet.header), sizeof(Header) - 2);

  packet.pitch = pitch;
  packet.yaw = yaw;
  packet.shoot = shoot;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  return encode_packet(out, capacity, packet);
}

size_t encode_nav(
  uint8_t * out, size_t capacity, float vx, float vy, float wz, float target_angle, uint8_t seq)
{
  NavigationPacket packet{};
  packet.header.header = 0xA5;
  packet.header.data_length = static_cast<uint16_t>(kNavDataSize);
  packet.header.seq = seq;
  packet.header.cmd_id = kCmdNavDownlink;
  crc8::Append_CRC8_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet.header), sizeof(Header) - 2);

  packet.vx = vx;
  packet.vy = vy;
  packet.wz = wz;
  packet.target_angle = target_angle;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  return encode_packet(out, capacity, packet);
}

bool decode_aim(const uint8_t * data, size_t size, AimFields & out)
{
  if (data == nullptr || size != sizeof(SendPacket)) return false;
  if (!crc8::Verify_CRC8_Check_Sum(data, sizeof(Header) - 2)) return false;
  if (!crc16::Verify_CRC16_Check_Sum(data, static_cast<uint32_t>(size))) return false;

  SendPacket packet{};
  std::memcpy(&packet, data, sizeof(packet));
  if (packet.header.cmd_id != kCmdAimDownlink || packet.header.data_length != kAimDataSize) {
    return false;
  }
  out.pitch = packet.pitch;
  out.yaw = packet.yaw;
  out.shoot = packet.shoot;
  return true;
}

bool decode_nav(const uint8_t * data, size_t size, NavFields & out)
{
  if (data == nullptr || size != sizeof(NavigationPacket)) return false;
  if (!crc8::Verify_CRC8_Check_Sum(data, sizeof(Header) - 2)) return false;
  if (!crc16::Verify_CRC16_Check_Sum(data, static_cast<uint32_t>(size))) return false;

  NavigationPacket packet{};
  std::memcpy(&packet, data, sizeof(packet));
  if (packet.header.cmd_id != kCmdNavDownlink || packet.header.data_length != kNavDataSize) {
    return false;
  }
  out.vx = packet.vx;
  out.vy = packet.vy;
  out.wz = packet.wz;
  out.target_angle = packet.target_angle;
  return true;
}

bool decode_aim_payload(const uint8_t * data, size_t size, AimFields & out)
{
  if (data == nullptr || size != kAimDataSize) return false;
  std::memcpy(&out.pitch, data, sizeof(out.pitch));
  std::memcpy(&out.yaw, data + sizeof(out.pitch), sizeof(out.yaw));
  std::memcpy(&out.shoot, data + sizeof(out.pitch) + sizeof(out.yaw), sizeof(out.shoot));
  return true;
}

bool decode_nav_payload(const uint8_t * data, size_t size, NavFields & out)
{
  if (data == nullptr || size != kNavDataSize) return false;
  std::memcpy(&out.vx, data, sizeof(out.vx));
  std::memcpy(&out.vy, data + sizeof(out.vx), sizeof(out.vy));
  std::memcpy(&out.wz, data + sizeof(out.vx) + sizeof(out.vy), sizeof(out.wz));
  std::memcpy(
    &out.target_angle, data + sizeof(out.vx) + sizeof(out.vy) + sizeof(out.wz),
    sizeof(out.target_angle));
  return true;
}

}  // namespace info_relay
