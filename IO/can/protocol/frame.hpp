/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__CAN__PROTOCOL__FRAME_HPP
#define INFO_RELAY__IO__CAN__PROTOCOL__FRAME_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "IO/can/protocol/crc.hpp"

namespace info_relay::io::can::protocol
{

// CAN 标准帧：11 位 ID + 最多 8 字节数据
struct CanFrame
{
  uint32_t id = 0;
  std::array<uint8_t, 8> data{};
  uint8_t dlc = 0;
};

// 字节流分片：前 7 字节参与 CRC，第 8 字节为校验值
struct CanChunk
{
  uint8_t seq = 0;    // 报文序号，用于丢弃迟到的旧分片
  uint8_t index = 0;  // 当前分片序号，从 0 开始
  uint8_t count = 1;  // 总分片数
  uint8_t len = 0;    // 本片有效数据长度，1~3
  uint8_t data[3]{};  // 有效载荷
  uint8_t crc = 0;    // crc8(seq..data)
};

static_assert(std::is_trivially_copyable_v<CanChunk>, "CanChunk 必须可直接映射到 CAN 数据场");
static_assert(sizeof(CanChunk) == 8, "CanChunk 必须正好占满一个 CAN 数据场");

inline CanChunk decode_chunk(const uint8_t * data)
{
  CanChunk chunk;
  chunk.seq = data[0];
  chunk.index = data[1];
  chunk.count = data[2];
  chunk.len = data[3];
  for (size_t i = 0; i < 3; ++i) chunk.data[i] = data[4 + i];
  chunk.crc = data[7];
  return chunk;
}

inline void encode_chunk(const CanChunk & chunk, uint8_t * data)
{
  data[0] = chunk.seq;
  data[1] = chunk.index;
  data[2] = chunk.count;
  data[3] = chunk.len;
  for (size_t i = 0; i < 3; ++i) data[4 + i] = chunk.data[i];
  data[7] = crc8(data, 7);
}

inline bool chunk_valid(const CanChunk & chunk)
{
  if (chunk.len == 0 || chunk.len > 3) return false;
  if (chunk.count == 0 || chunk.index >= chunk.count) return false;
  uint8_t raw[8];
  encode_chunk(chunk, raw);
  return raw[7] == chunk.crc;
}

}  // namespace info_relay::io::can::protocol

#endif  // INFO_RELAY__IO__CAN__PROTOCOL__FRAME_HPP
