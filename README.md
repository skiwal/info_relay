# info_relay

<p align="center">
  <img src="assets/logo.jpg" alt="Navigator Vision Team 队徽" width="220">
</p>

**战队 / Team：** Navigator Vision Team  
**代码来源 / Source：** Navigator Vision Team，本仓库自研代码的文件头均标注来源与 MIT 标识。

串口 / CAN 通信中转进程。整机唯一持有下位机通信链路的程序：解析协议后通过 iceoryx2 发布，
并接收任意外部进程写入的指令，通过同一通信链路发回下位机。

本仓库与视觉、导航工程完全解耦：不引用其源码、头文件、构建产物或运行产物，
也不维护 ROS 2 节点；ROS 2 与 iceoryx2 的桥接由导航侧自行负责。

## 项目结构

```text
src/                     relay 中转主流程：启动链路、装配 iceoryx2 节点、转发与节流
  relay.cpp/.hpp
main.cpp                 进程入口：配置、日志、退出信号，然后进入 src/relay
tools/                   通用工具：配置加载、日志、路径、退出信号、IPC 自检
IO/
  inc/IO.hpp             通信方式接口模板：init / open / read / write / close
  inc/board.hpp          通信板：持有 IO 实例，完成分帧、协议解析与状态搬运
  io_factory.cpp         按配置创建串口或 CAN 实现
  serial/                串口实现（基于 3rdparty/serial）
    protocol/            串口 CRC、帧结构与读写数据结构体
  can/                   SocketCAN 实现
    protocol/            CAN CRC、帧结构与读写数据结构体
msg/relay_msgs.hpp       对外 iceoryx2 线格式契约（与视觉/导航侧共用）
configs/                 运行配置
docs/                    设计说明
```

## 职责边界

| 负责 | 不负责 |
| --- | --- |
| 打开/关闭通信链路、断链重连 | 相机采集与推理 |
| 分帧、CRC、协议解析 | PnP、跟踪、瞄准决策 |
| 解析结果发布到 iceoryx2 | 录像与图像调试流 |
| 接收外部指令并通过通信链路写回下位机 | 开火判据（由视觉侧决定） |
| — | ROS 2 节点与桥接（导航侧自行维护） |
| — | 虚拟下位机、模拟数据与录制回放 |

## 依赖

编译期（Ubuntu，`sudo apt install`）：

```bash
build-essential cmake libyaml-cpp-dev libspdlog-dev libfmt-dev libeigen3-dev
```

跨进程通信库 iceoryx2 需**系统级安装**（与外部进程共用同一份）：

```bash
git clone --depth 1 --branch v0.10.0 https://github.com/eclipse-iceoryx/iceoryx2.git
cmake -S iceoryx2 -B iceoryx2/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
cmake --build iceoryx2/build --parallel          # 需要 Rust 工具链（cargo）
sudo cmake --install iceoryx2/build --prefix /usr/local
```

CAN 链路使用 Linux SocketCAN，需在系统中提前配置好网络接口（如 `can0`）；
当前 CAN 协议为 8 字节分片方案，具体 CAN ID 由 `configs/info_relay.yaml` 配置。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

产物：

| 路径 | 说明 |
| --- | --- |
| `build/info_relay` | 中转进程 |
| `build/ipc_selftest` | 依赖与共享内存自检 |

## 运行

```bash
./build/info_relay configs/info_relay.yaml
# 或省略参数，等价于使用 <仓库根>/configs/info_relay.yaml
./build/info_relay
```

自检（不启动业务，只验证 iceoryx2 通路）：

```bash
./build/ipc_selftest        # 期望输出 PASS
```

## 路径约定

所有运行产物默认落在本仓库内，与启动时的当前目录无关：

| 用途 | 默认位置 | 可否修改 |
| --- | --- | --- |
| 运行日志 | `<repo>/logs/<启动时刻>.log` | 配置 `log_dir` |
| 默认配置 | `<repo>/configs/info_relay.yaml` | 命令行参数 |
| 构建产物 | `<repo>/build/` | 由构建命令决定 |

仓库根按以下顺序确定：环境变量 `INFO_RELAY_HOME` → 可执行文件所在目录（在 `build/` 下则取其父目录）
→ 当前工作目录（并打印一次 warn）。因此从任意目录启动都写回仓库内。

## 与其他进程的契约

主题为 `<relay_prefix>/<topic>`，默认前缀 `rm_vision`。载荷为定长 POD，
定义见 `msg/relay_msgs.hpp`。所有主题对任何有需求的进程开放，不区分导航与自瞄。

| 主题 | 载荷 | 频率 | 方向 |
| --- | --- | --- | --- |
| `imu` | `Imu` | 500 Hz | relay → 外部 |
| `chassis` | `Chassis` | 100 Hz | relay → 外部 |
| `robot` / `game` | `Robot` / `Game` | 1 Hz | relay → 外部 |
| `health` | `Health` | 10 Hz | relay → 任意 |
| `cmd_aim` | `AimCmd` | 外部帧率 | 外部 → relay |
| `cmd_nav` | `NavCmd` | 外部帧率 | 外部 → relay |

`Health.schema_version` 用于运行期握手。当前 `kSchemaVersion = 3`：`source_kind` 语义已改为
通信链路类型（0 serial / 1 can），移除了 fake/replay。外部侧必须同步 `kSchemaVersion` 与字段语义，
否则链路会被判为契约不匹配。

## 许可证

本仓库自研代码由 Navigator Vision Team 所有，采用 **MIT License** 发布，完整条款见 [`LICENSE`](LICENSE)。
每个自研代码文件头部均带有 `SPDX-License-Identifier: MIT` 标识。

`3rdparty/serial` 为第三方代码，保留其上游版权声明与许可证；这部分文件不适用本仓库的上述 MIT 授权声明。
