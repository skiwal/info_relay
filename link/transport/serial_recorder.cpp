/*
 * 串口原始字节记录/回放的实现：私有二进制格式的写盘与读取。
 *
 * 文件布局（小端，与 x86_64 / aarch64 一致）：
 *   magic[8] "RMSERIAL" | start_unix_ns:u64 | count:u32 | 若干记录
 *   记录 = t_ns:u64 | dir:u8 | len:u32 | data[len]
 *
 * 时间戳统一取 info_relay::SessionClock 的相对纳秒，与相机录像共用同一条时间轴。
 *
 * 注意：契约写在 serial_recorder.hpp，本文件只写实现要点。
 */
#include "link/transport/serial_recorder.hpp"

#include <chrono>
#include <filesystem>

#include "common/logger.hpp"

namespace fs = std::filesystem;

namespace info_relay::sim
{
namespace
{
// 文件魔数：用于识别格式版本，读到时必须完全匹配
constexpr char k_serial_magic[8] = {'R', 'M', 'S', 'E', 'R', 'I', 'A', 'L'};

// 功能：创建目录，失败只告警
// 输入：
//   dir  目录路径
// 输出：原样返回 dir，便于链式使用
// 备注：只创建不删除；权限不足时写盘会失败，由调用方后续检查文件状态
std::string ensure_dir(const std::string & dir)
{
  try {
    fs::create_directories(dir);
  } catch (const std::exception & e) {
    logger()->warn("[REC] Failed to create dir {}: {}", dir, e.what());
  }
  return dir;
}
}  // namespace

// 实现要点：构造时立即写入三字段文件头并 flush，保证即使进程被强杀，
// 文件里也已经有可识别的 magic；count 先用 0 占位，close() 时回填。
SerialRecorder::SerialRecorder(const std::string & session_dir)
{
  ensure_dir(session_dir);
  path_ = session_dir + "/serial.bin";
  file_.open(path_, std::ios::binary | std::ios::trunc);
  if (!file_.is_open()) {
    logger()->warn("[REC] Cannot open serial record file: {}", path_);
    return;
  }

  const uint64_t start_unix_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
  uint32_t count = 0;
  file_.write(k_serial_magic, sizeof(k_serial_magic));
  file_.write(reinterpret_cast<const char *>(&start_unix_ns), sizeof(start_unix_ns));
  file_.write(reinterpret_cast<const char *>(&count), sizeof(count));
  file_.flush();

  logger()->info("[REC] Serial recording to {}", path_);
}

SerialRecorder::~SerialRecorder() { close(); }

void SerialRecorder::record_rx(const uint8_t * data, size_t size) { write_record(0, data, size); }

void SerialRecorder::record_tx(const uint8_t * data, size_t size) { write_record(1, data, size); }

// 实现要点：时间戳在锁外取，避免把时钟调用也算进临界区；每 256 条 flush 一次，
// 在"崩溃最多丢 256 条"和"频繁落盘拖慢串口线程"之间取折中。
void SerialRecorder::write_record(uint8_t dir, const uint8_t * data, size_t size)
{
  if (!file_.is_open() || !data || size == 0) return;
  std::lock_guard<std::mutex> lock(mutex_);

  const uint64_t t_ns = SessionClock::now_rel_ns();
  const uint32_t len = static_cast<uint32_t>(size);
  file_.write(reinterpret_cast<const char *>(&t_ns), sizeof(t_ns));
  file_.write(reinterpret_cast<const char *>(&dir), sizeof(dir));
  file_.write(reinterpret_cast<const char *>(&len), sizeof(len));
  file_.write(reinterpret_cast<const char *>(data), size);
  count_++;

  // 周期性落盘，避免崩溃时丢数据
  if (count_ % 256 == 0) file_.flush();
}

// 实现要点：关闭时把真实记录条数回填到文件头第二个字段之后的位置，
// 这样读取端既可以在不扫全文件的情况下知道记录数。
void SerialRecorder::close()
{
  if (closed_) return;
  closed_ = true;
  if (!file_.is_open()) return;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 回填记录条数
    file_.seekp(
      static_cast<std::streamoff>(sizeof(k_serial_magic) + sizeof(uint64_t)), std::ios::beg);
    file_.write(reinterpret_cast<const char *>(&count_), sizeof(count_));
    file_.flush();
    file_.close();
  }
  logger()->info("[REC] Serial recording closed ({} records): {}", count_, path_);
}

SerialReplayReader::SerialReplayReader(const std::string & path) : path_(path) {}

// 实现要点：读取端不信任文件内容——magic 不符、长度字段超过 1MB、读越界都立即终止，
// 避免被损坏的记录文件拖进无限循环或超大内存分配。
bool SerialReplayReader::load(std::string & error)
{
  std::ifstream in(path_, std::ios::binary);
  if (!in.is_open()) {
    error = "cannot open " + path_;
    return ok_ = false;
  }

  char magic[8] = {};
  uint64_t start_unix_ns = 0;
  uint32_t count = 0;
  in.read(magic, sizeof(magic));
  in.read(reinterpret_cast<char *>(&start_unix_ns), sizeof(start_unix_ns));
  in.read(reinterpret_cast<char *>(&count), sizeof(count));

  for (size_t i = 0; i < sizeof(k_serial_magic); ++i) {
    if (magic[i] != k_serial_magic[i]) {
      error = "bad magic in " + path_;
      return ok_ = false;
    }
  }

  records_.clear();
  rx_bytes_ = 0;
  while (in.good()) {
    SerialRecord rec;
    in.read(reinterpret_cast<char *>(&rec.t_ns), sizeof(rec.t_ns));
    if (!in.good()) break;
    in.read(reinterpret_cast<char *>(&rec.dir), sizeof(rec.dir));
    uint32_t len = 0;
    in.read(reinterpret_cast<char *>(&len), sizeof(len));
    if (!in.good() || len == 0 || len > (1u << 20)) break;
    rec.data.resize(len);
    in.read(reinterpret_cast<char *>(rec.data.data()), len);
    if (!in.good()) break;
    if (rec.dir == 0) rx_bytes_ += len;
    records_.push_back(std::move(rec));
  }

  ok_ = true;
  return true;
}

}  // namespace info_relay::sim
