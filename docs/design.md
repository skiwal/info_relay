# info_relay：通信中转进程

整机唯一的下位机通信持有者。本工程只维护一套 iceoryx2 中转系统，不接入 ROS 2；
外部进程通过 iceoryx2 共享内存交换数据，ROS 2 桥接由导航侧自行维护。

---

## 1. 分层结构

| 目录 | 职责 |
| --- | --- |
| `main.cpp` | 配置加载、日志初始化、退出信号，然后进入 `src/relay` |
| `src/` | relay 主流程：启动通信链路、装配 iceoryx2 节点、收发转发与节流 |
| `tools/` | 配置、日志、路径、退出信号、IPC 自检等通用工具 |
| `IO/inc/IO.hpp` | 通信接口模板：`init` / `open` / `read` / `write` / `close` |
| `IO/inc/board.hpp` | 通信板：持有 IO 实例，完成分帧、协议解析与状态搬运 |
| `IO/serial/` | 串口实现与 `protocol/` 下的 CRC、帧结构、读写数据结构 |
| `IO/can/` | SocketCAN 实现与 `protocol/` 下的 CRC、分片帧、读写数据结构 |
| `msg/relay_msgs.hpp` | 对外 iceoryx2 线格式契约 |

主流程严格按 `init → open → read` 启动通信链路；外部写入的指令经 `write` 发回下位机。
所有主题对任意有需求的进程开放，不区分导航与自瞄。

---

## 2. 职责边界

| info_relay 负责 | 外部进程负责 |
| --- | --- |
| 打开/关闭通信链路，断链重连 | 相机采集与 YOLO 推理 |
| 分帧、CRC、按 cmd_id 号段分派、赛季协议解析 | PnP 解算、跟踪滤波、瞄准决策 |
| 把解析结果发布到 iceoryx2 | 订阅上行数据、发布下行指令 |
| 接收外部指令并打包写回下位机 | 录像、图像调试流、开火判据 |

**不做**：任何视觉算法、坐标变换、滤波、图像处理；不引入 ROS 2 节点；
不提供虚拟下位机、模拟数据或录制回放。

---

## 3. 主题契约

主题为 `<relay_prefix>/<topic>`，默认前缀 `rm_vision`。载荷全部是定长 POD，
定义见 `msg/relay_msgs.hpp`，两侧共用同一份头文件，尺寸由 `static_assert` 固化。

| 主题 | 载荷 | 频率 | 方向 |
| --- | --- | --- | --- |
| `imu` | `Imu` | 500 Hz（有新帧才发） | relay → 外部 |
| `chassis` | `Chassis` | 100 Hz | relay → 外部 |
| `robot` | `Robot` | 1 Hz | relay → 外部 |
| `game` | `Game` | 1 Hz | relay → 外部 |
| `health` | `Health` | 10 Hz | relay → 任意 |
| `cmd_aim` | `AimCmd` | 外部帧率 | 外部 → relay |
| `cmd_nav` | `NavCmd` | 外部帧率 | 外部 → relay |

时间基准：所有 `t_ns` 都是本机 `steady_clock` 纳秒计数。同一台机器上可以直接比较，
**禁止在订阅侧重新打时间戳**，否则插值与延迟统计都会失真。

当前 `kSchemaVersion = 3`：`source_kind` 改为通信链路类型（0 serial / 1 can），
移除了 fake/replay 语义。外部侧必须同步版本与含义，否则握手会失败。

---

## 4. 通信输入与输出

**输入**：`IO::read(buf, size, timeout_ms)` 返回的字节流。

| 链路 | 来源 |
| --- | --- |
| `serial` | 真实串口设备，线路格式由 `IO/serial/protocol` 定义 |
| `can` | Linux SocketCAN，8 字节分片重组后交给同一协议解析逻辑 |

**输出**（解析成功才发布）：

| 输出 | 来源帧 | 失败语义 |
| --- | --- | --- |
| `Imu` | 0x0503 + 0x0500 缓存 | 校验失败丢帧，只在 `Health.crc_errors` 累加 |
| `Chassis` | 0x0502 | 同上 |
| `Robot` | 0x0201 + 0x0202 | 未收到时保持上次值，`valid_mask` 对应位为 0 |
| `Game` | 0x0001 | 未收到时 `valid=0`、`in_battle=0` |
| `Health` | 统计量 | 链路异常时照常发布，`transport_ok=0` |

---

## 5. 订阅端接收约定

| 项 | 约定 |
| --- | --- |
| 缓冲 | 每拍把队列排空，只保留最新一条；IMU 绝不能排队，否则延迟线性累积 |
| 下行 | 外部进程发布 `AimCmd` / `NavCmd`，relay 每拍取最新值写入通信链路 |
| 失链 | 超过 `relay_timeout_ms` 无上行帧 → `transport_ok=0`；开火判据由外部侧决定 |
| 启动顺序 | 两侧可任意顺序启动；relay 未起时外部进程正常运行，只是无上行数据 |
| 版本 | `kSchemaVersion` 不符时应拒绝启动（字段顺序变化会让两端静默错位） |

---

## 6. 启动与验证

```bash
./build/info_relay configs/info_relay.yaml
./build/ipc_selftest     # 期望输出 PASS
```

外部侧日志出现下面两行即说明链路通：

```
[RelayClient] 已连接主题前缀 rm_vision
[RelayClient] 收到首个 IMU 样本，链路建立
```

---

## 7. 依赖

iceoryx2 以系统库形式安装；CMake 从 `find_package(iceoryx2-cxx REQUIRED)` 解析。
串口实现使用仓库内的 `3rdparty/serial`；CAN 实现使用 Linux SocketCAN。

```bash
git clone --depth 1 --branch v0.10.0 https://github.com/eclipse-iceoryx/iceoryx2.git
cmake -S iceoryx2 -B iceoryx2/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
cmake --build iceoryx2/build --parallel
sudo cmake --install iceoryx2/build --prefix /usr/local
```
