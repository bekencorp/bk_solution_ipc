# PT Camera Frame Project (BK7259 SMP)

* [中文](./README_CN.md)

## 1 Overview

`pt_camera_frame` is a headless network-camera project based on BK7259 SMP. It
reuses the `smart_lock` doorbell stack for BLE provisioning, TCP/UDP streaming,
full-duplex audio, ASR, and AP-powerdown keepalive. Its default video pipeline is:

```text
SC3336/SC3336P (MIPI CSI, 2304×1296@15fps)
    │
    ▼
ISP MP (1280×720, NV12, frame mode, 3 buffers)
    │  zero-copy dequeue
    ▼
Temporal NDR (automatic ISO/motion gate)
    │
    ▼
H.264 frame-mode HW encoder ──► TCP/UDP network stream
```

The project does not initialize an LCD/GPU display pipeline and targets headless
IPC and pan-tilt cameras.

## 2 Features

| Module | Default configuration |
| --- | --- |
| Camera | SC3336/SC3336P, MIPI CSI, maximum sensor input 2304×1296@15fps |
| ISP MP | 1280×720 NV12, frame mode, three ISP frame buffers |
| H.264 | Frame-mode HW encode, GOP 40, Balanced QP preset at 1.5 Mbps |
| NDR | Three-slot PSRAM temporal-denoise output with two-stage processing/encode pipeline |
| Day/night | Automatic luminance detection, IR LED, dual-coil IR-CUT, grayscale night image |
| Network | Doorbell LAN TCP/UDP streaming |
| Provisioning | BLE Boarding |
| Audio | ADK, AEC v3, G.711/G.722, on-board microphone/speaker |
| ASR | KWS, TFLite-Micro, NPU, SRAM mode |
| Low power | AP powerdown with CP TCP keepalive and RTC wakeup |
| Pan-tilt tracking | Optional face detection and motor tracking; disabled in the default defconfig |

## 3 Video and NDR

### 3.1 Frame-mode zero-copy pipeline

With `CONFIG_PT_MP_H264_FRAME_MODE=y`:

1. ISP MP produces 1280×720 NV12 frames in frame mode.
2. The project takes ownership of MP with `BK_CAM_IOCTL_CHANNEL_ACQUIRE`.
3. The NDR bond manages ISP frames through `BK_CAM_IOCTL_FRAME_POP/QBUF` without a full-frame memcpy.
4. Three uncoded-PSRAM output slots allow the history frame, encoded frame, and next NDR output to coexist.
5. The encode thread submits contiguous NV12 through `BK_H264_ENCODE_IOCTL_SET_INPUT_BUF`.

When `CONFIG_PT_MP_H264_FRAME_MODE` is disabled, the project falls back to the
common doorbell Flexa ISP-to-H.264 pipeline.

### 3.2 Automatic NDR gate

The default mode is `auto`:

- NDR is allowed only when the current ISO is greater than 400.
- NDR is disabled when the previous encoded frame has more than 2000 intra 8×8 CUs, indicating significant motion.
- The default current-frame weight is `alpha=100/256`.
- When NDR is off, the ISP frame is submitted directly without copying.

Runtime controls are available through the serial CLI:

```text
pt_dnr status
pt_dnr auto
pt_dnr on
pt_dnr off
pt_dnr alpha <1..256>
pt_dnr iso <0..5000>
pt_dnr intra <value>
```

The settings apply to the current run only and are not persisted.

## 4 Automatic day/night switching

After the video pipeline starts, the `pt_day_night` task samples ISP exposure
luminance:

- Three consecutive samples below 30 switch to infrared night mode.
- Three consecutive samples above 369 restore color day mode.
- Sampling is every 100 ms while deciding and every second after stabilization.
- Five rapid transitions within 60 seconds trigger a 180-second lockout to prevent IR-CUT chatter.

Night mode sets ISP saturation to zero before enabling the IR LED and switching
IR-CUT. Day mode reverses the hardware sequence and restores the sensor's default
CPROC settings.

## 5 Default hardware configuration

Pin assignments are defined in `ap/ap_main.c`, `ap/src/pt_ir_led.c`, and
`ap/src/pt_ircut.c`.

| Signal | GPIO | Notes |
| --- | --- | --- |
| MIPI I2C SCL | GPIO_64 | I2C bus 1 |
| MIPI I2C SDA | GPIO_65 | |
| Sensor Reset | GPIO_60 | |
| Sensor XCLK | GPIO_59 | |
| IR LED | GPIO_49 | Active high |
| IR-CUT Night | GPIO_47 | 300 ms pulse |
| IR-CUT Day | GPIO_48 | 300 ms pulse |

Verify active levels and the IR-CUT coil driver before porting to different hardware.

## 6 Key configuration

| Configuration | Default | Purpose |
| --- | --- | --- |
| `CONFIG_PT_MP_H264_FRAME_MODE` | y | Frame-mode H.264, zero-copy NDR, and automatic day/night switching |
| `CONFIG_PT_TRACKING` | n (defconfig) | Face detection and pan-tilt motor tracking |
| `CONFIG_CSI_SC3336/SC3336P` | y | Default MIPI sensor |
| `CONFIG_BK_ENCODER_H264_FRAME_TASK_PRIORITY` | 1 | Frame encoder control-task priority |
| `CONFIG_BK_ENCODER_HW_TASK_PRIORITY` | 1 | Encoder hardware-worker priority |
| `CONFIG_H264_QP_PRESET_BALANCED` | y | Default H.264 bitrate/QP preset |
| `CONFIG_BK_H264E_DEBUG_SEI` | y | H.264 debug SEI |
| `CONFIG_MDS_SNAPSHOT` | y | Snapshot support |

## 7 Build

This is a solution project. Build from its project directory and point `SDK_DIR`
to the AVDK SDK:

```bash
cd bk_solution_ipc_release_4.0.1/projects/pt_camera_frame
make bk7259 \
    SDK_DIR=/absolute/path/to/bk_avdk_smp_release_4.0.1 \
    -j32
```

Do not pass `PROJECT=pt_camera_frame`. The firmware is generated at:

```text
build/bk7259/pt_camera_frame/package/all-app.bin
```

## 8 Demo

1. Flash `all-app.bin` and connect the SC3336/SC3336P MIPI camera, IR LED, and IR-CUT.
2. Add a Video Doorbell device in the BekenIot APK and complete BLE provisioning to a 2.4 GHz Wi-Fi network.
3. Start video to receive a 1280×720 H.264 network stream.
4. Run `pt_dnr status` and select `auto`, `on`, or `off` as required.
5. Change ambient illumination and verify automatic color-day and infrared-grayscale-night transitions.

## 9 Layout

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

Shared solution components are under `../../components/`. Low-level drivers and
encoders come from the AVDK SDK selected by `SDK_DIR`.
