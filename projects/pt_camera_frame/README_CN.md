# PT Camera Frame 工程（BK7259 SMP）

* [English](./README.md)

## 1 项目概述

`pt_camera_frame` 是基于 BK7259 SMP 的无屏网络摄像头工程。工程复用 `smart_lock`
门铃业务栈，包括 BLE 配网、TCP/UDP 图传、双向音频、ASR 和 AP powerdown
低功耗保活，并将默认视频链路切换为：

```text
SC3336/SC3336P（MIPI CSI，2304×1296@15fps）
    │
    ▼
ISP MP（1280×720，NV12，整帧模式，3-buffer）
    │  零拷贝取帧
    ▼
时域 NDR（自动 ISO/运动门控）
    │
    ▼
H.264 整帧硬编码 ──► TCP/UDP 网络图传
```

工程默认不初始化 LCD/GPU 显示链路，适用于无本地屏幕的 IPC 和云台摄像机。

## 2 主要功能

| 模块 | 默认配置 |
| --- | --- |
| 摄像头 | SC3336/SC3336P，MIPI CSI，传感器最大输入 2304×1296@15fps |
| ISP MP | 1280×720 NV12，整帧模式，3 个 ISP frame buffer |
| H.264 | 整帧硬编码，GOP 40，Balanced QP preset 默认码率 1.5 Mbps |
| NDR | 三槽 PSRAM 时域降噪输出，处理与编码双线程流水 |
| 日夜切换 | 自动亮度检测、IR LED、双线圈 IR-CUT、夜间灰度图像 |
| 网络 | doorbell LAN TCP/UDP 图传 |
| 配网 | BLE Boarding |
| 音频 | ADK、AEC v3、G.711/G.722、板载 Mic/Speaker |
| ASR | KWS、TFLite-Micro、NPU，SRAM 模式 |
| 低功耗 | AP powerdown，CP TCP keepalive 和 RTC 唤醒 |
| 云台跟踪 | 可选人脸检测和电机跟踪，默认 defconfig 关闭 |

## 3 视频与 NDR

### 3.1 整帧零拷贝链路

`CONFIG_PT_MP_H264_FRAME_MODE=y` 时：

1. ISP MP 使用整帧模式输出 1280×720 NV12。
2. 工程通过 `BK_CAM_IOCTL_CHANNEL_ACQUIRE` 接管 MP 通道。
3. NDR bond 通过 `BK_CAM_IOCTL_FRAME_POP/QBUF` 管理 ISP frame，不执行整帧 memcpy。
4. NDR 输出使用三个独立的 uncoded PSRAM 槽：历史帧、编码帧和下一帧输出可以并行占用。
5. 编码线程通过 `BK_H264_ENCODE_IOCTL_SET_INPUT_BUF` 向 H.264 整帧编码器提交连续 NV12。

关闭 `CONFIG_PT_MP_H264_FRAME_MODE` 后，工程回退到公共 doorbell Flexa
ISP→H.264 链路。

### 3.2 NDR 自动门控

默认模式为 `auto`：

- 当前 ISO 大于 400 时才允许开启 NDR。
- 上一编码帧的 intra 8×8 CU 数大于 2000 时判定运动较大，关闭 NDR。
- 默认当前帧权重 `alpha=100/256`。
- NDR 关闭时直接零拷贝提交 ISP frame。

运行时可通过串口 CLI 调整：

```text
pt_dnr status
pt_dnr auto
pt_dnr on
pt_dnr off
pt_dnr alpha <1..256>
pt_dnr iso <0..5000>
pt_dnr intra <value>
```

参数只在当前运行周期生效，不写入持久化存储。

## 4 自动日夜切换

视频链路启动后，`pt_day_night` 任务周期读取 ISP 曝光亮度：

- 亮度连续 3 次低于 30：切换到夜间红外模式。
- 亮度连续 3 次高于 369：恢复日间彩色模式。
- 稳定后每 1 秒采样，切换判定阶段每 100 ms 采样。
- 60 秒内频繁切换达到 5 次时，锁定 180 秒以抑制 IR-CUT 抖动。

夜间模式先将 ISP saturation 置 0，再开启 IR LED 并切换 IR-CUT；日间模式按相反
顺序恢复传感器默认 CPROC。

## 5 默认硬件配置

引脚定义位于 `ap/ap_main.c`、`ap/src/pt_ir_led.c` 和 `ap/src/pt_ircut.c`。

| 信号 | GPIO | 说明 |
| --- | --- | --- |
| MIPI I2C SCL | GPIO_64 | I2C bus 1 |
| MIPI I2C SDA | GPIO_65 | |
| Sensor Reset | GPIO_60 | |
| Sensor XCLK | GPIO_59 | |
| IR LED | GPIO_49 | 高电平开启 |
| IR-CUT Night | GPIO_47 | 300 ms 脉冲 |
| IR-CUT Day | GPIO_48 | 300 ms 脉冲 |

移植到其他硬件前必须核对电平有效性和 IR-CUT 线圈驱动电路。

## 6 关键配置

| 配置项 | 默认值 | 作用 |
| --- | --- | --- |
| `CONFIG_PT_MP_H264_FRAME_MODE` | y | 启用整帧 H.264、零拷贝 NDR 和自动日夜切换 |
| `CONFIG_PT_TRACKING` | n（defconfig） | 启用人脸检测和云台电机跟踪 |
| `CONFIG_CSI_SC3336/SC3336P` | y | 启用默认 MIPI Sensor |
| `CONFIG_BK_ENCODER_H264_FRAME_TASK_PRIORITY` | 1 | 整帧编码控制任务优先级 |
| `CONFIG_BK_ENCODER_HW_TASK_PRIORITY` | 1 | 编码硬件 worker 优先级 |
| `CONFIG_H264_QP_PRESET_BALANCED` | y | 默认 H.264 码率/QP preset |
| `CONFIG_BK_H264E_DEBUG_SEI` | y | 启用 H.264 调试 SEI |
| `CONFIG_MDS_SNAPSHOT` | y | 启用抓拍能力 |

## 7 编译

本工程属于 solution，必须在工程目录中通过 `SDK_DIR` 指向 AVDK SDK：

```bash
cd bk_solution_ipc_release_4.0.1/projects/pt_camera_frame
make bk7259 \
    SDK_DIR=/absolute/path/to/bk_avdk_smp_release_4.0.1 \
    -j32
```

不要传入 `PROJECT=pt_camera_frame`。编译产物：

```text
build/bk7259/pt_camera_frame/package/all-app.bin
```

## 8 演示流程

1. 烧录 `all-app.bin`，连接 SC3336/SC3336P MIPI 摄像头、IR LED 和 IR-CUT。
2. 使用 BekenIot APK 添加可视门铃设备并完成 2.4 GHz Wi-Fi BLE 配网。
3. 开启视频后，设备输出 1280×720 H.264 网络码流。
4. 使用 `pt_dnr status` 查看 NDR 参数，按需切换 auto/on/off。
5. 改变环境亮度，确认日间彩色与夜间红外灰度模式能够自动切换。

## 9 工程目录

```text
projects/pt_camera_frame
├── ap/
│   ├── ap_main.c
│   ├── Kconfig.projbuild
│   ├── config/bk7259_ap/defconfig
│   ├── include/
│   └── src/
│       ├── pt_camera_frame_bringup.c
│       ├── pt_frame_ndr_bond.c
│       ├── pt_day_night.c
│       ├── pt_ir_led.c
│       ├── pt_ircut.c
│       └── pt_motor.c
├── cp/
├── partitions/bk7259/
├── CMakeLists.txt
└── Makefile
```

公共 solution 组件位于 `../../components/`，底层驱动和编码器由 `SDK_DIR` 指向的
AVDK SDK 提供。
