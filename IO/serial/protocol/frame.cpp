/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/serial/protocol/frame.hpp"

#include <algorithm>
#include <cstring>

#include "tools/comm_warning.hpp"

namespace referee_comm
{

size_t Receiver::Encode(
  uint8_t * out, size_t capacity, uint8_t seq, uint16_t cmd_id, const void * payload,
  uint16_t length)
{
  if (out == nullptr || length > MAX_PAYLOAD) return 0;
  const size_t total = HEADER_SIZE + CMD_ID_SIZE + static_cast<size_t>(length) + TAIL_SIZE;
  if (total > capacity) return 0;

  out[0] = SOF;
  out[1] = static_cast<uint8_t>(length & 0xFF);
  out[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
  out[3] = seq;
  Append_CRC8_Check_Sum(out, HEADER_SIZE);

  out[HEADER_SIZE] = static_cast<uint8_t>(cmd_id & 0xFF);
  out[HEADER_SIZE + 1] = static_cast<uint8_t>((cmd_id >> 8) & 0xFF);
  if (length > 0 && payload != nullptr)
    std::memcpy(out + HEADER_SIZE + CMD_ID_SIZE, payload, length);

  Append_CRC16_Check_Sum(out, static_cast<uint32_t>(total));
  return total;
}

void Receiver::append(uint8_t byte)
{
  if (available() == MAX_PACKET) {
    ++begin_;
    ++stats_.garbage_bytes;
  }
  buffer_[end_ % MAX_PACKET] = byte;
  ++end_;
}

const uint8_t * Receiver::view(size_t offset, size_t length, uint8_t * scratch) const
{
  const size_t start = (begin_ + offset) % MAX_PACKET;
  if (start + length <= MAX_PACKET) return buffer_ + start;
  for (size_t i = 0; i < length; ++i) {
    scratch[i] = buffer_[(begin_ + offset + i) % MAX_PACKET];
  }
  return scratch;
}

void Receiver::Feed(const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) return;

  // 与下位机一致：整段输入即视为对端存活证据，与帧本身是否有效无关。
  has_rx_ = true;
  last_rx_ = std::chrono::steady_clock::now();

  uint8_t scratch[MAX_PACKET];
  for (size_t i = 0; i < size; ++i) {
    append(data[i]);

    while (available() > 0) {
      if (at(0) != SOF) {
        Resync();
        continue;
      }

      if (available() < HEADER_SIZE) break;
      const size_t payload =
        static_cast<size_t>(at(1)) | (static_cast<size_t>(at(2)) << 8);
      const uint16_t raw_cmd = available() >= HEADER_SIZE + CMD_ID_SIZE
                                 ? static_cast<uint16_t>(
                                     static_cast<uint16_t>(at(HEADER_SIZE)) |
                                     (static_cast<uint16_t>(at(HEADER_SIZE + 1)) << 8))
                                 : 0;
      const uint8_t * bytes = view(0, HEADER_SIZE, scratch);
      if (!Verify_CRC8_Check_Sum(bytes, HEADER_SIZE) || payload > MAX_PAYLOAD) {
        if (payload > MAX_PAYLOAD) {
          stats_.payload_errors++;
          info_relay::comm_warnings().report(
            info_relay::CommWarningKind::PayloadTooLong, raw_cmd, MAX_PAYLOAD,
            static_cast<uint32_t>(payload), "serial_header");
        } else {
          stats_.crc8_errors++;
          info_relay::comm_warnings().report(
            info_relay::CommWarningKind::Crc8Failure, raw_cmd, HEADER_SIZE, 0, "serial_header");
        }
        Resync();
        continue;
      }
      const size_t total = HEADER_SIZE + CMD_ID_SIZE + payload + TAIL_SIZE;
      if (available() < total) break;
      bytes = view(0, total, scratch);
      if (!Verify_CRC16_Check_Sum(bytes, static_cast<uint32_t>(total))) {
        stats_.crc16_errors++;
        info_relay::comm_warnings().report(
          info_relay::CommWarningKind::Crc16Failure, raw_cmd, static_cast<uint32_t>(total),
          static_cast<uint32_t>(available()), "serial_frame");
        Resync();
        continue;
      }

      Frame frame{};
      frame.cmd_id = static_cast<uint16_t>(bytes[5]) | (static_cast<uint16_t>(bytes[6]) << 8);
      frame.data = bytes + HEADER_SIZE + CMD_ID_SIZE;
      frame.length = static_cast<uint16_t>(payload);
      stats_.frames++;
      protocol_.OnCommand(frame);
      Discard(total);
    }
  }
}

void Receiver::Reset()
{
  begin_ = 0;
  end_ = 0;
}

void Receiver::Discard(size_t size)
{
  const size_t n = std::min(size, available());
  begin_ += n;
  if (begin_ == end_) {
    begin_ = 0;
    end_ = 0;
  }
}

void Receiver::Resync()
{
  size_t next = 1;
  const size_t count = available();
  while (next < count && at(next) != SOF) ++next;
  stats_.garbage_bytes += next;
  Discard(next);
}

}  // namespace referee_comm
