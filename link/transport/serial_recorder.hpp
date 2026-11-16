#ifndef INFO_RELAY__LINK__TRANSPORT__SERIAL_RECORDER_HPP
#define INFO_RELAY__LINK__TRANSPORT__SERIAL_RECORDER_HPP

/*
 * 串口原始字节记录与回放：按「相对会话起点的时间戳 + 方向 + 长度」把 rx/tx 字节落盘，
 * 回放时还原到同一时间轴，用于没有下位机时复现现场问题。
 *
 * 数据流：IByteTransport 的收发字节 → SerialRecorder → serial.bin；
 *         回放时 serial.bin → SerialReplayReader → 虚拟传输层按时间戳推字节。
 *
 * 线程：SerialRecorder 的写盘函数会被串口收发线程并发调用，内部用 mutex 串行；
 *       SerialReplayReader 只在构造阶段单线程读取，加载完成后只读。
 *
 * 注意：文件是私有二进制格式（magic "RMSERIAL" + 起始 UNIX 时间 + 记录条数 + 若干记录），
 *       时间戳用 info_relay::SessionClock 的相对纳秒，与相机录像共用同一条时间轴，
 *       因此串口与相机回放可以按同一时间基准对齐。
 *       namespace 沿用 info_relay::sim，避免牵动 transport / virtual_mcu / quaternion_source
 *       里已有的限定名；后续如需整体改名可单独提交。
 */

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "common/clock.hpp"

namespace info_relay::sim
{

// 一条串口字节记录
struct SerialRecord
{
  uint64_t t_ns = 0;  // 相对会话起点的时刻，ns
  uint8_t dir = 0;    // 方向：0 = rx(下位机→视觉)，1 = tx(视觉→下位机)
  std::vector<uint8_t> data;  // 该时刻收到的原始字节
};

// 串口原始字节记录器：每收到或发出一段字节就写一条记录
class SerialRecorder
{
public:
  // 功能：打开（或创建）会话目录下的 serial.bin 并写入文件头
  // 输入：
  //   session_dir  会话目录路径；不存在时自动创建
  // 输出：无
  // 备注：打不开文件时只告警不抛异常，后续写盘静默跳过
  explicit SerialRecorder(const std::string & session_dir);

  // 功能：关闭文件并回填记录条数
  // 输入：无
  // 输出：无
  // 备注：析构会自动调用；重复调用安全
  ~SerialRecorder();

  // 功能：记录一段下行（下位机 → 视觉）字节
  // 输入：
  //   data  字节缓冲指针，不能为空
  //   size  字节数，0 时直接返回
  // 输出：无
  // 备注：内部加锁并周期性 flush，可在串口接收线程直接调用
  void record_rx(const uint8_t * data, size_t size);

  // 功能：记录一段上行（视觉 → 下位机）字节，参数语义同 record_rx
  // 输入：
  //   data  字节缓冲指针，不能为空
  //   size  字节数，0 时直接返回
  // 输出：无
  // 备注：内部加锁，可在串口发送路径直接调用
  void record_tx(const uint8_t * data, size_t size);

  // 功能：关闭记录文件并回填总记录数
  // 输入：无
  // 输出：无
  // 备注：重复调用安全；关闭后 write_record 静默返回
  void close();

  // 返回值：serial.bin 的完整路径
  const std::string & path() const { return path_; }

private:
  // 功能：按方向写一条记录
  // 输入：
  //   dir   方向，0 = rx，1 = tx
  //   data  字节缓冲，为空或 size 为 0 时跳过
  //   size  字节数
  // 输出：无
  // 备注：调用方无需加锁；内部每 256 条 flush 一次，避免崩溃丢数据
  void write_record(uint8_t dir, const uint8_t * data, size_t size);

  std::string path_;      // serial.bin 的完整路径，构造期确定
  std::ofstream file_;    // 二进制输出流，只在本类内使用
  std::mutex mutex_;      // 保护 file_ 与 count_，收发线程并发写
  uint32_t count_ = 0;    // 已写入的记录条数，关闭时回填到文件头
  bool closed_ = false;   // 是否已关闭，避免重复 close 触发 seekp
};

// 串口原始字节回放读取器：一次性把 serial.bin 读进内存
class SerialReplayReader
{
public:
  // 功能：构造读取器，只记录路径不读文件
  // 输入：
  //   path  serial.bin 路径
  // 输出：无
  // 备注：实际读取发生在 load()
  explicit SerialReplayReader(const std::string & path);

  // 功能：把整个记录文件读入内存
  // 输入：
  //   error  失败时写入可读原因
  // 输出：true 表示加载成功；false 时 records() 为空
  // 备注：magic 不符或记录长度异常时停止读取并按失败处理
  bool load(std::string & error);

  // 返回值：是否已成功加载
  bool ok() const { return ok_; }

  // 返回值：按时间顺序排列的记录列表
  const std::vector<SerialRecord> & records() const { return records_; }

  // 返回值：rx 方向的累计字节数
  size_t rx_bytes() const { return rx_bytes_; }

private:
  std::string path_;             // 源文件路径
  bool ok_ = false;              // 加载是否成功
  std::vector<SerialRecord> records_;  // 全部记录
  size_t rx_bytes_ = 0;          // rx 方向的累计字节数
};

}  // namespace info_relay::sim

#endif  // INFO_RELAY__LINK__TRANSPORT__SERIAL_RECORDER_HPP
