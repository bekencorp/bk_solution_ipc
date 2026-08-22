# Beken BK7259 IPC Solution

- [中文](./README_CN.md)

## Overview

**BK7259 IPC Solution** is Beken's open-source smart network camera solution, based on the **BK7259** SoC and the Armino base SDK **BK_AVDK_SMP**. It provides example projects for headless IPC, AOV low power, and ISP/H264E image-quality tuning. The solution supports MIPI CSI capture, H.264 hardware encoding, Wi-Fi streaming, BLE provisioning, two-way audio with AEC, keyword spotting (KWS/ASR), and AP-powerdown low-power keepalive, with peripherals such as a MIPI CSI camera and speaker/mic.

## Documentation

- [BK7259 IPC Solution Online Docs](https://docs.bekencorp.com/arminodoc/bk_ipc/bk7259/en/v4.0.1/index.html)
- [Armino SMP SDK (BK AVDK SMP)](https://docs.bekencorp.com/arminodoc/bk_avdk_smp/smp_doc/bk7259/en/v4.0.1/index.html)

## Hardware

This solution runs on the BK7259 development board with BK7259 SoC, MIPI CSI camera module, and speaker/mic audio.

- Development Board Hardware Reference (TBD)
- [BK7259 Datasheet](https://docs.bekencorp.com/spec/BK7259/BK7259_Datasheet.pdf)

Development kit purchase link: coming soon.

## Version Policy

This solution uses a **maintenance branch + release tag** model:

- `release/v4.0.1` is the ongoing maintenance branch for feature updates and fixes. It always has the latest code but may include changes not yet fully release-tested.
- `release/v4.0.1.x` tags are official releases (e.g. `release/v4.0.1.4`, `release/v4.0.1.6`), fully tested and suitable for production.
- The BK7259 IPC solution and Armino SMP SDK **must use the exact same tag**. For example, if the solution uses `release/v4.0.1.7`, the SDK must also use `release/v4.0.1.7`.

For development and production, pick the highest `release/v4.0.1.x` tag. Use the `release/v4.0.1` branch only when you need the latest features or are contributing to development.

## Get the Code

The BK7259 IPC solution and Armino SMP SDK are published on GitHub, Gitee, and GitLab. Choose a mirror based on your network and access.

BK7259 IPC solution:

- GitHub: <https://github.com/bekencorp/bk_solution_ipc>
- Gitee: <https://gitee.com/bekencorp/bk_solution_ipc>
- GitLab: <https://gitlab.bekencorp.com/armino/smp_solution/bk_solution_ipc>

Armino SMP SDK:

- GitHub: <https://github.com/bekencorp/bk_avdk_smp>
- Gitee: <https://gitee.com/bekencorp/bk_avdk_smp>
- GitLab: <https://gitlab.bekencorp.com/armino/bk_avdk_smp>

GitHub and Gitee are publicly accessible. GitLab is for enterprise customers only; contact your FAE or sales representative for access.

**Windows users**: Before cloning with Git for Windows, disable automatic CRLF conversion to avoid build failures from scripts or source files being converted to CRLF. Not required on Linux, macOS, or WSL.

```bash
git config --global core.autocrlf false
```

If you already cloned the repos, changing this setting will not fix existing files; re-clone after updating the config.

The example below uses GitHub and the `release/v4.0.1.7` tag. **When fetching code, always pick the latest release tag and use the same tag for both the solution and SDK.** Replace the URLs when using other mirrors.

```bash
mkdir -p ~/armino && cd ~/armino

# Armino SMP SDK
git clone --branch release/v4.0.1.7 https://github.com/bekencorp/bk_avdk_smp.git

# BK7259 IPC solution
git clone --branch release/v4.0.1.7 https://github.com/bekencorp/bk_solution_ipc.git
```

See [Armino SMP Get Started](https://docs.bekencorp.com/arminodoc/bk_avdk_smp/smp_doc/bk7259/en/v4.0.1/get-started/index.html) for more details.

## Build Environment Setup

Armino SMP supports local and Docker builds. Choose one of the following for your platform.

### Linux (local)

Run the environment setup script in the SDK directory:

```bash
# Script is inside the bk_avdk_smp repo
cd ~/armino/bk_avdk_smp
sudo bash tools/env_tools/setup/armino_env_setup.sh
```

### Windows (local)

Download and install [Armino Bash](https://dl.bekencorp.com/tools/arminosdk/WindowsInstaller/Armino-Bash-Setup_0.3.0.exe).

### Docker

Docker image: [`bekencorp/armino-idk`](https://hub.docker.com/r/bekencorp/armino-idk/tags). Use tag `1.5` or newer. Supports Windows, Linux, and macOS.

For detailed setup steps, see [Armino SMP Get Started: Environment Setup and Build](https://docs.bekencorp.com/arminodoc/bk_avdk_smp/smp_doc/bk7259/en/v4.0.1/get-started/index.html).

## Build a Project

Using the `ipc` project (`projects/ipc`) as an example, point `SDK_DIR` at the Armino SMP SDK and build locally:

```bash
cd ~/armino/bk_solution_ipc/projects/ipc
make bk7259 SDK_DIR=~/armino/bk_avdk_smp PROJECT=ipc
```

Docker build is also supported (`./dbuild.sh` on Linux/macOS, `.\dbuild.ps1` on Windows PowerShell):

```bash
cd ~/armino/bk_solution_ipc/projects/ipc
export SDK_DIR=~/armino/bk_avdk_smp
./dbuild.sh make bk7259 PROJECT=ipc
```

After a successful build, the flashable firmware is at (relative to the `bk_solution_ipc/` repo root):

```text
projects/ipc/build/bk7259/ipc/package/all-app.bin
```

For other projects, replace the path and `PROJECT` value with the matching `projects/<name>`.

## Flash Firmware

You can flash firmware using either method below:

- Download and use the [BKFIL local flash tool](https://dl.bekencorp.com/tools/bkfil/v4)
- Use the [BKFIL web flash tool](https://connect.aclsemi.com/)

Select the `all-app.bin` built in the previous section.

For detailed flashing steps, see [Armino SMP Get Started](https://docs.bekencorp.com/arminodoc/bk_avdk_smp/smp_doc/bk7259/en/v4.0.1/get-started/index.html).

## Reference Projects

| Project | Main Features | Details |
| --- | --- | --- |
| [ipc](../projects/ipc/) | Headless IPC camera: SC3336 2304×1296 capture, H.264 streaming, BLE provisioning, two-way audio, low-power keepalive. | [Detailed description and usage guide](https://docs.bekencorp.com/arminodoc/bk_ipc/bk7259/en/v4.0.1/projects/ipc/index.html) |
| [aov](../projects/aov/) | AOV low-power IPC: CP/AP state machine, motion detection and event wake (WIP). | [Detailed description and usage guide](https://docs.bekencorp.com/arminodoc/bk_ipc/bk7259/en/v4.0.1/projects/aov/index.html) |
| [isp_h264_tuning](../projects/isp_h264_tuning/) | PC-side image tuning: Wi-Fi preview of ISP raw frames and H.264 encoded streams. | [Detailed description and usage guide](https://docs.bekencorp.com/arminodoc/bk_ipc/bk7259/en/v4.0.1/projects/isp_h264_tuning/index.html) |

For project selection and differences, see the online [Example Projects](https://docs.bekencorp.com/arminodoc/bk_ipc/bk7259/en/v4.0.1/projects/index.html) page.

## BEKEN Resources

- [BEKEN Official Site](https://www.bekencorp.com/)
- [ARMINO Developer Forum](https://armino.bekencorp.com/)
- [BEKEN Documentation Center](https://docs.bekencorp.com/)
- WeChat Channels: 博通集成电路
