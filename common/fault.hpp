#ifndef INFO_RELAY__COMMON__FAULT_HPP
#define INFO_RELAY__COMMON__FAULT_HPP

/*
 * 异常注入：虚拟下位机用它制造丢包、延迟、跳变、校验错误与噪声，
 * 用来验证串口链路与视觉侧在恶劣条件下的行为。
 *
 * 数据流：配置 fault: 段 → FaultConfig → FaultInjector 在产帧时按概率逐项施加。
 *
 * 依赖与边界：纯头文件；只读 YAML，不碰真实串口与硬件。
 *
 * 使用注意：随机数使用固定种子，同一配置下异常序列可复现；
 *           全部率为 0 时 enabled() 为 false，生产路径不产生额外分支开销。
 */

#include <yaml-cpp/yaml.h>

#include <random>

#include "common/yaml.hpp"

namespace info_relay
{

// 串口侧的异常注入参数
struct FaultConfig
{
  double imu_drop_rate = 0.0;         // 丢包概率 [0,1]
  double imu_delay_ms = 0.0;          // 额外延迟，ms
  int imu_spike_every = 0;            // 每 N 帧注入一次跳变，0 表示关闭
  double imu_spike_rad = 1.5708;      // 跳变幅度，rad
  double imu_corrupt_crc_rate = 0.0;  // 校验错误概率 [0,1]
  double imu_noise_rad = 0.0;         // 角度噪声标准差，rad

  // 功能：判断是否有任何注入项被启用
  // 输入：无
  // 输出：true 表示至少一项概率或频率大于 0
  // 备注：为 false 时调用方可跳过整段注入逻辑
  bool enabled() const
  {
    return imu_drop_rate > 0 || imu_delay_ms > 0 || imu_spike_every > 0 ||
           imu_corrupt_crc_rate > 0 || imu_noise_rad > 0;
  }

  // 功能：从配置的 fault: 段读取参数
  // 输入：
  //   yaml  根节点；无 fault: 段时全部取默认值
  // 输出：填好的配置
  // 备注：单项类型不符时该项回落默认值，不影响其余项
  static FaultConfig load(const YAML::Node & yaml)
  {
    FaultConfig cfg;
    const auto node = yaml["fault"];
    if (!node) return cfg;

    cfg.imu_drop_rate = value<double>(node, "imu_drop_rate", cfg.imu_drop_rate);
    cfg.imu_delay_ms = value<double>(node, "imu_delay_ms", cfg.imu_delay_ms);
    cfg.imu_spike_every = value<int>(node, "imu_spike_every", cfg.imu_spike_every);
    cfg.imu_spike_rad = value<double>(node, "imu_spike_rad", cfg.imu_spike_rad);
    cfg.imu_corrupt_crc_rate = value<double>(node, "imu_corrupt_crc_rate", cfg.imu_corrupt_crc_rate);
    cfg.imu_noise_rad = value<double>(node, "imu_noise_rad", cfg.imu_noise_rad);
    return cfg;
  }
};

// 注入器：持有配置与固定种子的随机源
class FaultInjector
{
public:
  // 功能：构造注入器
  // 输入：
  //   cfg   注入参数
  //   seed  随机种子，同一种子可复现同一序列
  // 输出：无
  // 备注：种子固定是为了让问题可复现，不要用时间戳做种子
  FaultInjector(const FaultConfig & cfg, int seed) : cfg_(cfg), rng_(static_cast<uint32_t>(seed)) {}

  // 返回值：注入参数
  const FaultConfig & config() const { return cfg_; }

  // 功能：以概率 p 返回 true
  // 输入：
  //   p  概率 [0,1]；<=0 恒 false，>=1 恒 true
  // 输出：是否命中
  // 备注：边界值走短路，不消耗随机数，保证 p=0 时序列完全不变
  bool chance(double p)
  {
    if (p <= 0.0) return false;
    if (p >= 1.0) return true;
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p;
  }

  // 功能：取标准正态分布随机数
  // 输入：无
  // 输出：均值 0、标准差 1 的样本
  // 备注：乘以噪声标准差即为实际注入量
  double gauss() { return normal_(rng_); }

private:
  FaultConfig cfg_;                                   // 注入参数
  std::mt19937 rng_;                                  // 固定种子的随机源
  std::normal_distribution<double> normal_{0.0, 1.0};  // 标准正态分布
};

}  // namespace info_relay

#endif  // INFO_RELAY__COMMON__FAULT_HPP
