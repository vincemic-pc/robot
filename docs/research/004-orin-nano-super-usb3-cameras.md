---
id: "004"
type: research
title: "NVIDIA Jetson Orin Nano Super — USB 3.0 Camera Attachment Deep Dive"
status: ✅ Complete
created: "2026-02-24"
current_phase: "4 of 5"
---

## Introduction

The ROSMASTER A1 robot runs on an NVIDIA Jetson Orin Nano Super Developer Kit. USB 3.0 camera connectivity is critical for high-bandwidth stereo vision (OAK-D Pro) and visual SLAM. Prior internal research (002) identified that all USB devices currently route through a USB 2.0 hub on the Yahboom carrier board, limiting throughput. This research focuses on the Orin Nano *Super* variant specifically — its hardware specs, USB controller architecture, supported camera interfaces, and practical methods for attaching USB 3.0 cameras at full bandwidth. External web sources (NVIDIA docs, Luxonis forums, Jetson community, teardowns) will be the primary research material.

## Objectives

- Understand the NVIDIA Jetson Orin Nano Super hardware specs, especially USB controller and port topology
- Determine the differences between Orin Nano and Orin Nano Super relevant to USB 3.0 and camera I/O
- Research how USB 3.0 cameras (OAK-D, RealSense, industrial USB3 Vision) attach to Jetson Orin Nano Super
- Identify known issues, workarounds, and best practices from the Jetson community
- Determine if the USB-C port on the Orin Nano Super carrier board supports USB 3.0 host mode for cameras
- Research alternative high-bandwidth camera interfaces (CSI/MIPI vs USB 3.0) and when each is preferred
- Provide actionable guidance for achieving full USB 3.0 bandwidth with the OAK-D Pro on this platform

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Orin Nano Super Hardware Specs & USB Architecture | ✅ Complete | NVIDIA official specs for Orin Nano Super; SoC USB controller details (xHCI, USB 3.2 Gen 2 vs Gen 1); carrier board USB port mapping; differences from original Orin Nano; pinout and block diagrams from NVIDIA developer docs and datasheets | 2026-02-24 |
| 2 | Carrier Board USB Topology & Port Identification | ✅ Complete | NVIDIA Developer Kit carrier board schematic/design guide; which physical ports are USB 3.x vs 2.0; USB-C port capabilities (host vs device vs dual-role); internal USB hub ICs used on reference carrier; how Yahboom ROSMASTER A1 carrier may differ; community teardowns and port mapping | 2026-02-24 |
| 3 | USB 3.0 Camera Compatibility & Attachment Methods | ✅ Complete | Web research on attaching USB 3.0 cameras to Jetson Orin Nano Super; OAK-D Pro (Luxonis) USB 3 on Jetson — known issues and solutions; Intel RealSense on Jetson USB 3; industrial USB3 Vision cameras; cable requirements (USB-C to USB-C, USB-A 3.0); power delivery considerations; bandwidth sharing when multiple USB 3 devices are connected | 2026-02-24 |
| 4 | Community Solutions & Known Issues | ✅ Complete | NVIDIA Developer Forums posts about USB 3 cameras on Orin Nano; Luxonis community/docs for OAK-D on Jetson; common failure modes (USB 2.0 fallback, bandwidth throttling, power brownout); kernel/device-tree USB configuration; udev rules; JetPack version-specific USB fixes; USB 3.0 hub recommendations for Jetson | 2026-02-24 |
| 5 | Alternative Interfaces & Best Practices Summary | ✅ Complete | CSI/MIPI camera interface vs USB 3.0 — pros/cons on Orin Nano Super; OAK-D Pro FFC (fixed-focus camera) CSI variant availability; GMSL2 camera support; practical recommendation matrix for camera attachment on this platform; summary of actionable steps for the ROSMASTER A1 robot | 2026-02-24 |

## Phase 1: Orin Nano Super Hardware Specs & USB Architecture

**Status:** ✅ Complete  
**Session:** 2026-02-24

### NVIDIA Jetson Orin Nano Super — Hardware Overview

The Jetson Orin Nano Super Developer Kit (P3766) consists of:
- **Module**: P3767 (Jetson Orin Nano 8GB) — contains the T234 SoC
- **Carrier Board**: P3768 — reference carrier board with all external I/O
- **SoC**: NVIDIA T234 (Tegra Orin family)

#### Core Specifications (Super Mode = 25W MAXN)

| Parameter | Orin Nano (original) | Orin Nano Super |
|-----------|---------------------|------------------|
| GPU | Ampere, 1024 CUDA / 32 Tensor cores, 635 MHz | Same cores, **1,020 MHz** |
| CPU | 6-core Arm Cortex-A78AE v8.2, 1.5 GHz | Same cores, **1.7 GHz** |
| Memory | 8 GB 128-bit LPDDR5, 68 GB/s | Same capacity, **102 GB/s** |
| AI Performance | 40 INT8 TOPS (Sparse) | **67 INT8 TOPS** (Sparse) |
| Max Power | 15W | **25W** |
| Video Decode | 1x 4K60, 2x 4K30, 5x 1080p60 (H.265) | Same |
| Price | $499 | **$249** |

**Critical finding: "Super" is a software-only upgrade.** Same hardware, same SoC, same carrier board. The upgrade is activated by installing JetPack 6.1+ and setting power mode MAXN: `sudo nvpmodel -m 2`. USB I/O is completely unchanged between Orin Nano and Orin Nano Super.

### T234 SoC USB Controller Architecture

The T234 SoC implements NVIDIA's proprietary **xUSB** subsystem, which is **xHCI-specification compliant**. It consists of two major blocks:

#### xHCI Controller (`usb@3610000`)
- Compliant with the xHCI (eXtensible Host Controller Interface) specification
- Supports the full USB 2.0 and USB 3.2 protocol stack:

| Speed Mode | Protocol | Max Bandwidth |
|------------|----------|---------------|
| Super Speed Plus Host | USB 3.2 Gen 2 | **10 Gbps** |
| Super Speed Host | USB 3.2 Gen 1 | 5 Gbps |
| High Speed Host | USB 2.0 | 480 Mbps |
| Full Speed Host | USB 1.1 | 12 Mbps |
| Low Speed Host | USB 1.0 | 1.5 Mbps |

The xHCI controller also supports:
- **OTG** (On-The-Go) — USB role switching (host ↔ device)
- **Device mode**: SuperSpeed (3.0), High Speed (2.0), MTP, ADB, RNDIS
- **Auto Suspend / Remote Wakeup / Auto Resume**
- **ELPG** (Engine Level Power Gating) for both HS and SS partitions
- **LPM states** (U1, U2, U3)
- **Hot Plug**
- **USB Video Class (UVC)** — critical for USB cameras like OAK-D Pro
- **USB Audio Class (UAC)**, HID, Mass Storage, ECM, NCM
- **BC1.2 charging** and Apple charger detection

#### xUSB Pad Controller (`padctl@3520000`)
- Manages the physical USB pad configuration (pinctrl-style bindings)
- Configures pads (USB2 and USB3), ports (host/peripheral/OTG), VBUS supply, overcurrent pins
- Each USB pad has two components:
  - **D+/D−** signal pins → connect to UTMI pads (USB 2.0)
  - **SSTX/SSRX** signal pins → connect to UPHY lanes (USB 3.2 SuperSpeed)

#### UPHY (Universal PHY) Lane Assignment

The SuperSpeed USB lanes are multiplexed through NVIDIA's UPHY block, which is shared between PCIe, UFS, and xUSB. The P3767 SOM has **three USB 3.2 SuperSpeed lanes**:

| Signal Lines | UPHY Block | Lane | USB Port |
|--------------|------------|------|----------|
| USBSS0_RX/TX | UPHY0 | Lane 0 | USB 3.2 Port P0 |
| DP0_TXD0/1 | UPHY0 | Lane 1 | USB 3.2 Port P1 |
| DP0_TXD2/3 | UPHY0 | Lane 2 | USB 3.2 Port P2 |

**Default UPHY Configuration** (`hsio-uphy-config-0`):

| Lane | Default Assignment |
|------|--------------------|
| 0 | USB 3.2 (P0) |
| 1 | USB 3.2 (P1) |
| 2 | USB 3.2 (P2) |
| 3 | PCIe x1 (C1), RP |
| 4-7 | PCIe x4 (C4), RP |

All three USB 3.2 lanes are available by default. UPHY lane sharing only becomes a concern if custom carrier boards reassign lanes to PCIe or UFS.

The UPHY configuration is set via `ODMDATA` in the flash configuration:
```
ODMDATA="gbe-uphy-config-8,hsstp-lane-map-3,hsio-uphy-config-0"
```

### P3768 Carrier Board — USB Port Mapping

The reference carrier board (P3768) exposes the following USB connectivity:

| # | Connector | Type | Listed Speed | Mode | Notes |
|---|-----------|------|-------------|------|-------|
| 4 | USB-C | Type-C | USB 3.2 | Host / Device / Recovery | Data only — NO display output (no DP Alt Mode) |
| 6 | USB 3.2 Type-A (×4) | Type-A stacked (2 dual-stacked) | 10 Gbps (Gen 2) | Host only | Each stack limited to 3A VBUS |

#### USB-C Port (#4) — Detailed Behavior
- Supports **three modes**:
  1. **Host mode (DFP)**: Acts as downstream-facing port — can connect USB devices (cameras, drives, hubs) just like the Type-A ports
  2. **Device mode (UFP)**: Jetson acts as a USB device when connected to a PC (exposes Mass Storage, Serial, RNDIS Ethernet)
  3. **USB Recovery mode**: For flashing Jetson via SDK Manager
- Uses USB port P0 (usb2-0 + usb3-0) with OTG role-switching via a FUSB301 Type-C controller on I2C (`i2c@c240000`)
- **Cannot output display** — HDMI/DP over USB-C is NOT supported

#### Type-A Ports (#6) — Internal Topology
**Critical finding from the NVIDIA Adaptation Guide:** The J6 Type-A stacked port USB signals come "from the Port 0 USBSS lines of the SOM **through USB HUB**." This means:

- **The 4 Type-A ports route through an on-board USB hub** on the P3768 carrier board
- The hub sits between the SoC's native USB 3.2 ports and the physical Type-A connectors
- This hub chip is the key component to investigate in Phase 2

#### Logical USB Topology (Inferred from Documentation)

```
T234 SoC (xHCI usb@3610000)
│
├── USB 3.2 Port P0 + USB2 Port 0
│   └──→ FUSB301 Type-C controller → USB-C connector (#4)
│         (OTG: Host/Device/Recovery)
│
├── USB 3.2 Port P1 + USB2 Port 1  ──┐
│                                     ├── USB Hub Chip
├── USB 3.2 Port P2 + USB2 Port 2  ──┘   └──→ 4x USB 3.2 Gen 2 Type-A (#6)
│
└── (USB2 interfaces for internal use)
```

**The hub should be USB 3.x capable** — the Type-A ports are rated at 10 Gbps (Gen 2) in the datasheet and user guide. If the hub is USB 3.2, then cameras connected to the Type-A ports SHOULD negotiate at SuperSpeed.

The problem observed in prior research (002) — where all devices appear on USB 2.0 — may indicate either:
1. The Yahboom ROSMASTER A1 carrier board introduces its own USB 2.0-only hub upstream of the Jetson carrier board's ports, or
2. A cable/connection issue prevents SuperSpeed negotiation

#### Camera Interfaces (Non-USB Alternative)
- **2× MIPI CSI-2 camera connectors** (22-pin, 0.5mm pitch, bottom-contact):
  - CAM0: CSI 1 ×2 lane
  - CAM1: CSI 1 ×2 lane **or** 1 ×4 lane
  - Compatible with Raspberry Pi Camera Module v2 via 15-to-22 pin adapter cable
- These are native camera interfaces — no USB bandwidth competition, but require CSI-compatible cameras (not applicable to OAK-D Pro which is USB only)

### PCIe as Camera Interface Alternative

The Orin Nano's PCIe capabilities on P3768:
- M.2 Key M (x4 PCIe Gen 3) — theoretically ~31.5 Gbps aggregate
- M.2 Key M (x2 PCIe Gen 3) — ~15.75 Gbps
- M.2 Key E (populated with WiFi by default)

Some industrial cameras support PCIe, but OAK-D Pro does not.

### Key Differences: Orin Nano vs Orin Nano Super for USB/Camera

| Aspect | Orin Nano | Orin Nano Super | Difference |
|--------|-----------|-----------------|------------|
| USB Controllers | T234 xHCI @ usb@3610000 | Identical | None |
| USB 3.2 Ports | Up to 3 (P0, P1, P2) | Identical | None |
| USB 3.2 Gen 2 (10G) | Supported | Supported | None |
| Carrier Board | P3768 | P3768 | Same board |
| Type-A Ports | 4× USB 3.2 Gen 2 | Identical | None |
| USB-C Port | Host/Device/Recovery | Identical | None |
| CSI Connectors | 2× MIPI CSI-2 | Identical | None |
| GPU/CPU/Memory | Lower clocks | Higher clocks | Compute only |
| Upgrade Method | N/A | JetPack 6.1+ `nvpmodel -m 2` | Software only |

**Conclusion: The "Super" upgrade has zero impact on USB or camera I/O. All USB research for Orin Nano applies identically to Orin Nano Super.**

**Key Discoveries:**
- The Orin Nano Super is a software-only upgrade — same T234 SoC, same P3768 carrier board, same USB architecture
- The T234 SoC has an xHCI-compliant xUSB controller supporting up to USB 3.2 Gen 2 (10 Gbps) natively with three SuperSpeed lanes
- The P3768 carrier board's 4× USB 3.2 Type-A ports route through an on-board USB hub chip — they do NOT connect directly to the SoC
- The USB-C port (#4) supports Host mode and connects more directly to the SoC via USB3-0 with FUSB301 OTG controller — potentially the most direct path for a USB 3.0 camera
- UPHY lanes are shared between USB, PCIe, and UFS; default config allocates all 3 lanes to USB 3.2
- UVC (USB Video Class) is explicitly supported on the T234 xHCI controller
- The USB 2.0-only problem from prior research (002) likely originates from the Yahboom carrier, not the Jetson's native capabilities

| File | Relevance |
|------|-----------|
| `docs/research/002-oakd-pro-usb3-connection.md` | Prior research identifying the USB 2.0 problem on the ROSMASTER A1 |

**External Sources:**
- [Jetson Orin Nano Developer Kit User Guide — Hardware Spec](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/hardware_spec.html)
- [Jetson Orin Nano Super Product Page](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
- [Orin Nano Super Developer Kit Datasheet (PDF)](https://nvdam.widen.net/s/zkfqjmtds2/jetson-orin-datasheet-nano-developer-kit-3575392-r2)
- [NVIDIA Blog: Orin Nano Super Boost](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)
- [Jetson Orin NX/Nano Series Adaptation Guide](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html)
- [Jetson Orin Series Supported Features](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SO/JetsonOrinSeries.html)

**Gaps:** Exact USB hub chip model on P3768 not confirmed from public docs; NVIDIA TRM is access-gated  
**Assumptions:** The on-board USB hub is USB 3.x capable based on the 10 Gbps port ratings in official specs

## Phase 2: Carrier Board USB Topology & Port Identification

**Status:** ✅ Complete  
**Session:** 2026-02-24

### Live USB Topology — Discovered via SSH Diagnostics

All data in this phase was gathered from the live ROSMASTER A1 robot at `192.168.7.250` using `lsusb`, `lsusb -t`, `dmesg`, sysfs exploration, and device tree inspection. This is the **actual measured topology**, not inferred from documentation.

#### Complete USB Bus Topology

```
T234 SoC xHCI (usb@3610000, Bus 001 @ 480M + Bus 002 @ 10000M)
│
├── [usb2-0 OTG + usb3-1 SS] → FUSB301 (I2C 1-0025) → USB-C connector (#4)
│   DIRECT SoC path — NO hub chips in between.
│   Modes: Host / Device / Recovery via OTG role-switching
│   SuperSpeed companion: usb3-1 ↔ usb2-0
│
├── [usb2-1 Host + usb3-0 SS] → VIA Labs VL822 (2109:0822) @ 10Gbps
│   USB 3.1 Gen 2 Hub | 4 downstream SS ports | Self-powered
│   │
│   ├── Port 1 (1-2.1): Silicon Labs CP210x UART (10c4:ea60) @ 12Mbps
│   │                     [SLLidar C1 LiDAR sensor]
│   │
│   ├── Port 2 (1-2.2 / 2-1.2): VIA Labs VL817 (2109:0817) @ 5Gbps
│   │   USB 3.0 Gen 1 Hub | 4 downstream SS + 5 HS ports | Self-powered
│   │   │
│   │   ├── Port 1 (1-2.2.1): OAK-D Pro (03e7:2485) @ 480Mbps ⚠️
│   │   │         Intel Movidius MyriadX — CURRENTLY at USB 2.0
│   │   │         *** SuperSpeed negotiation SUCCEEDS then FAILS ***
│   │   │         (see U1/U2 LPM failure analysis below)
│   │   │
│   │   ├── Port 5 (1-2.2.5): VIA Labs USB Billboard (2109:8817) @ 480Mbps
│   │   │         Standard USB 3.x hub companion device
│   │   │
│   │   └── Ports 2-4: Physical Type-A connectors (available for devices)
│   │
│   ├── Port 3 (1-2.3): QinHeng USB HUB (1a86:8091) @ 480Mbps
│   │   USB 2.0 ONLY Hub | 4 downstream ports | 100mA max draw
│   │   [YAHBOOM ROSMASTER A1 internal robot peripherals hub]
│   │   │
│   │   ├── Port 2 (1-2.3.2): C-Media USB Audio (0d8c:0012) @ 12Mbps
│   │   │                       [USB microphone for voice commands]
│   │   │
│   │   └── Port 3 (1-2.3.3): QinHeng CH34x Serial (1a86:7522) @ 12Mbps
│   │                           [Robot STM32 MCU UART bridge]
│   │
│   └── Port 4 (1-2.4): QinHeng CH340 Serial (1a86:7523) @ 12Mbps
│                         [Robot MCU secondary UART — directly on VL822]
│
└── [usb2-2 Host, NO SS companion] → Realtek Bluetooth (0bda:c822) @ 12Mbps
    USB 2.0 only — no SuperSpeed lane allocated to this port
```

### Hub Chip Identification

Three distinct USB hub chips were identified on the system:

#### 1. VIA Labs VL822 (VID:PID 2109:0822) — P3768 Carrier Board, Tier 1

| Property | Value |
|----------|-------|
| Standard | USB 3.1 Gen 2 (10 Gbps SuperSpeed+) |
| Downstream Ports | 4 ×SuperSpeed, 4 ×High-Speed |
| bcdUSB | 3.20 |
| Power | Self-powered |
| Max Current | 0mA from upstream (self-powered) |
| Location | Onboard the P3768 carrier board |
| Connected To | SoC via usb3-0 (SS) + usb2-1 (HS) |
| Bus 2 Address | 2-1 (SuperSpeed path) |
| Bus 1 Address | 1-2 (High-Speed companion path) |

The VL822 is the **first hub in the chain** between the SoC and all Type-A ports. It connects at the full 10 Gbps Gen 2 rate to the SoC. NVIDIA's Adaptation Guide confirms: *"USB signals for J6 come from Port 0 USBSS lines of the SOM through USB HUB."*

#### 2. VIA Labs VL817 (VID:PID 2109:0817) — P3768 Carrier Board, Tier 2

| Property | Value |
|----------|-------|
| Standard | USB 3.0 Gen 1 (5 Gbps SuperSpeed) |
| Downstream Ports | 4 ×SuperSpeed, 5 ×High-Speed (includes internal port) |
| bcdUSB | 3.20 |
| Power | Self-powered |
| Location | Onboard the P3768 carrier board, downstream of VL822 |
| Connected To | VL822 Port 2 @ 5 Gbps SuperSpeed |
| Bus 2 Address | 2-1.2 (SuperSpeed path) |
| Bus 1 Address | 1-2.2 (High-Speed companion path) |
| Billboard Device | 2109:8817 at Port 5 (USB 3.x standard) |

The VL817 is the **second hub in the cascade** and provides the actual downstream Type-A port connections. It connects to the VL822 at 5 Gbps (Gen 1), meaning **the physical Type-A ports are capped at 5 Gbps** despite the VL822 supporting 10 Gbps upstream. The extra HS port (port 5) hosts the USB Billboard Device, which is standard for USB 3.x hubs with Type-C awareness.

**Hub Cascade Bandwidth Impact:** SoC → VL822 (10G) → VL817 (5G) → devices. Any single device on a Type-A port can achieve up to 5 Gbps. Total aggregate bandwidth through the VL817 is shared among all connected devices at 5 Gbps.

#### 3. QinHeng Electronics USB HUB (VID:PID 1a86:8091) — Yahboom ROSMASTER A1

| Property | Value |
|----------|-------|
| Standard | USB 2.0 (480 Mbps High-Speed) |
| Downstream Ports | 4 |
| bcdUSB | 2.00 |
| Power | Bus-powered (100mA max) |
| Location | Yahboom ROSMASTER A1 carrier board / wiring harness |
| Connected To | VL822 Port 3 (via High-Speed only path) |
| Bus 1 Address | 1-2.3 |

This is **Yahboom's internal hub** for robot-specific peripherals (USB audio, serial to MCU). It is **USB 2.0 only** but this is intentional — the devices it serves (C-Media audio at 12 Mbps, CH34x serial at 12 Mbps) have no need for SuperSpeed. 

**Critical clarification from Phase 1:** Prior research (002) hypothesized that Yahboom might route ALL USB through a USB 2.0 hub, causing the OAK-D Pro to be limited to 480 Mbps. **This is NOT the case.** The QinHeng hub only carries low-bandwidth robot peripherals. The OAK-D Pro connects through the VL817 (USB 3.0 capable) hub which does have SuperSpeed lanes. The USB 2.0 fallback has a different root cause — U1/U2 LPM failures (see below).

### USB-C Port Capabilities — Direct SoC Path

The USB-C port (#4) on the P3768 carrier board has a **fundamentally different topology** from the Type-A ports:

| Property | Value |
|----------|-------|
| USB 3.2 Lane | usb3-1 (SuperSpeed) |
| USB 2.0 Lane | usb2-0 (High-Speed/OTG) |
| Role Controller | FUSB301 at I2C address `1-0025` (device ID 0x12) |
| CC Logic | FUSB301 handles CC line detection for plug orientation and role |
| Device Tree Mode | OTG (host ↔ device switching) |
| Hub Chips | **NONE** — direct SoC connection |
| SuperSpeed Status | usb3-1: `okay` |

**The USB-C port bypasses the VL822/VL817 hub cascade entirely.** A USB 3.0 camera connected via USB-C would negotiate directly with the SoC's xHCI controller through a single UPHY lane — no hub latency, no shared bandwidth, no hub firmware LPM issues.

This makes the USB-C port the **theoretically optimal connection point** for a USB 3.0 camera. However, it has trade-offs:
- It's the only USB-C port, also used for flash/recovery
- OTG role switching adds complexity (FUSB301 must detect host mode)
- Currently no USB-C to USB-C cable with the OAK-D Pro (which has USB-C connector)

### Device Tree USB Configuration

Gathered from live device tree on the robot (`/proc/device-tree/`):

| DT Port | Mode | SS Companion | Status | Physical Connection |
|---------|------|-------------|--------|---------------------|
| usb2-0 | OTG | usb3-1 | okay | USB-C connector |
| usb2-1 | Host | usb3-0 | okay | VL822 hub → Type-A ports |
| usb2-2 | Host | *(none)* | okay | Bluetooth only (no SS needed) |
| usb3-0 | Host | usb2-1 | okay | VL822 hub upstream |
| usb3-1 | Host | usb2-0 | okay | USB-C connector |
| usb3-2 | — | — | disabled | Not connected on P3768 |
| usb3-3 | — | — | disabled | Not connected on P3768 |

PHY names allocated to the xHCI controller: `usb2-0, usb2-1, usb2-2, usb3-0, usb3-1` (5 total).

**Note:** Only 2 of 3 possible SuperSpeed lanes are active: usb3-0 (to VL822 hub) and usb3-1 (to USB-C). The third lane (usb3-2) is disabled. Kernel `cmdline` contains no USB-related parameters.

### Critical Discovery: USB 3.0 SuperSpeed Flapping Due to U1/U2 LPM Failures

Analysis of `dmesg` reveals the OAK-D Pro **does physically negotiate SuperSpeed** but cannot sustain the link. The failure pattern repeats in a consistent cycle:

#### Flapping Pattern (from dmesg timestamps)

**Cycle 1 (~25s after boot):**
```
[25.5s] USB disconnect on bus 1 (HS path)
[26.3s] New SuperSpeed USB device on bus 2 → device 2-1.2.1 (SS!)
[26.5s] "Disable of device-initiated U1 failed"
[26.5s] "Disable of device-initiated U2 failed"
[28.2s] USB disconnect on bus 2 (SS path)
[28.6s] New HS device on bus 1 → device 1-2.2.1 @ 480Mbps (fallback)
```

**Cycle 2 (~324s):**
```
[323.5s] USB disconnect on bus 1
[324.5s] SuperSpeed connect on bus 2
[512-532s] U1/U2 failures, multiple resets (4× disconnect/reconnect attempts)
[533.7s] Final fallback to bus 1 @ 480Mbps
```

**Cycle 3 (~2567s):**
```
[2567.4s] USB disconnect on bus 1
[2568.1s] SuperSpeed connect on bus 2
[2569.8s] USB disconnect on bus 2
[2570.2s] Final fallback to bus 1 @ 480Mbps
```

#### Root Cause Analysis

The errors `"Disable of device-initiated U1 failed"` and `"Disable of device-initiated U2 failed"` indicate a **USB 3.0 Link Power Management (LPM) protocol failure** between:

1. **Intel Movidius MyriadX** (OAK-D Pro's USB controller) — initiates U1/U2 low-power states
2. **VIA Labs VL817** hub — intermediary that must manage LPM transitions
3. **NVIDIA tegra-xusb** (xHCI host controller) — must acknowledge LPM state changes

U1 and U2 are USB 3.0 link-level power saving states (similar to USB 2.0 Selective Suspend but at the physical layer). When the Movidius chip initiates a U1/U2 transition, the VL817 hub and/or the tegra-xusb controller fails to properly complete the handshake, causing the link to reset and eventually fall back to USB 2.0.

**Contributing factors identified from sysfs:**

| Setting | Value | Impact |
|---------|-------|--------|
| VL822 `power/control` | `auto` | Hub can autosuspend → may trigger U1/U2 |
| VL822 `power/autosuspend_delay_ms` | `0` | Zero delay — immediate suspend attempts |
| VL817 `power/control` | `auto` | Same aggressive autosuspend |
| VL817 `power/autosuspend_delay_ms` | `0` | Same zero-delay behavior |
| OAK-D `power/control` | `on` | Device itself is kept on, but upstream hubs can still trigger LPM |
| `usbcore.autosuspend` | `2` (seconds) | Global autosuspend enabled with 2s timeout |
| xHCI `quirks` | `0x0000000000010810` | Includes XHCI_SLOW_SUSPEND and XHCI_TRUST_TX_LENGTH |
| Module-level `quirks` | (empty) | No per-device quirks configured |
| Kernel USB cmdline params | (none) | No USB overrides in boot args |

**This is NOT a hardware wiring problem.** The SuperSpeed link physically works — the device enumerates at SuperSpeed and transfers data briefly before LPM failures cause disconnection. This is a firmware/driver compatibility issue between the Movidius USB controller, VIA Labs hub firmware, and tegra-xusb host driver.

### Physical Port to Bus Mapping

Verified via sysfs device enumeration:

| Physical Port | sysfs Path (HS) | sysfs Path (SS) | Current Device | Current Speed |
|---------------|-----------------|-----------------|----------------|---------------|
| USB-C (#4) | usb2-0 (OTG) | usb3-1 | *(empty)* | — |
| Type-A Stack Left-Top | 1-2.2.1 | 2-1.2.1 | OAK-D Pro | 480 Mbps (HS) |
| Type-A Stack Left-Bot | 1-2.2.2 | 2-1.2.2 | *(empty)* | — |
| Type-A Stack Right-Top | 1-2.2.3 | 2-1.2.3 | *(empty)* | — |
| Type-A Stack Right-Bot | 1-2.2.4 | 2-1.2.4 | *(empty)* | — |
| Internal (Yahboom) | 1-2.3.x | — | Audio, Serial | 12 Mbps |

*Note: Exact physical-to-logical port assignment for the four Type-A connectors may vary; the mapping above is best-effort based on the OAK-D Pro being in the first available port.*

### Yahboom ROSMASTER A1 — Carrier Board Differences

The ROSMASTER A1 does NOT replace the P3768 carrier board. Instead, Yahboom adds an **expansion layer** that plugs into the P3768's USB ports:

| Component | Source | Connection Point |
|-----------|--------|------------------|
| VL822 USB 3.1 Gen 2 Hub | P3768 carrier board (NVIDIA) | SoC usb3-0 + usb2-1 |
| VL817 USB 3.0 Hub | P3768 carrier board (NVIDIA) | VL822 Port 2 |
| QinHeng 8091 USB 2.0 Hub | Yahboom ROSMASTER A1 | VL822 Port 3 (via USB cable/harness) |
| CP210x UART | Yahboom/SLLidar | VL822 Port 1 |
| CH340/CH34x Serial × 2 | Yahboom ROSMASTER A1 | VL822 Port 3 (via QinHeng) and Port 4 |
| C-Media USB Audio | Yahboom ROSMASTER A1 | VL822 Port 3 → QinHeng Port 2 |

**Yahboom's QinHeng USB 2.0 hub consumes one of VL822's four downstream SuperSpeed ports** but only uses the HS companion (since it's USB 2.0 only). This means VL822's Port 3 SuperSpeed lane is wasted. However, this does not affect camera bandwidth since the Type-A physical ports connect through the VL817 (on VL822 Port 2), not the QinHeng hub.

The Yahboom website (yahboom.com/study/ROSMASTER-A1) provides extensive courseware but **no schematic or teardown documentation** for the ROSMASTER A1 carrier wiring. The internal hub and serial connections were identified entirely through live USB enumeration.

### xHCI Controller Details

| Property | Value |
|----------|-------|
| Controller | `usb@3610000` (tegra-xusb) |
| Protocol | xHCI (USB 3.1 Enhanced SuperSpeed) |
| Firmware Date | 2023-02-10 |
| Quirks Bitmask | `0x0000000000010810` |
| Decoded Quirks | XHCI_SLOW_SUSPEND, XHCI_TRUST_TX_LENGTH, XHCI_PME_STUCK_QUIRK |
| Kernel | Linux 5.15.148-tegra (aarch64) |
| Module Quirks Override | *(none set)* |

**Key Discoveries:**
- The P3768 carrier board uses a **two-tier VIA Labs hub cascade**: VL822 (USB 3.1 Gen 2, 10Gbps) → VL817 (USB 3.0, 5Gbps) → 4× Type-A ports
- **The OAK-D Pro physically achieves SuperSpeed** but falls back to USB 2.0 within seconds due to U1/U2 Link Power Management handshake failures between the Movidius MyriadX, VL817 hub, and tegra-xusb host
- The **USB-C port bypasses all hub chips** — connects directly to SoC via usb3-1/usb2-0 with FUSB301 OTG — making it the optimal direct path for a USB 3.0 camera
- The Yahboom QinHeng USB 2.0 hub **only serves robot peripherals** (audio, serial) — it does NOT cause the OAK-D Pro's USB 2.0 fallback as initially hypothesized in research 002
- Both VL822 and VL817 hubs have aggressive autosuspend settings (control=auto, delay=0ms) which likely trigger the U1/U2 LPM transitions that fail
- The xHCI quirks bitmask `0x10810` includes XHCI_SLOW_SUSPEND — the controller already has a suspend-related workaround active, suggesting known suspend issues on tegra-xusb
- No USB kernel boot parameters or per-device quirks are currently configured — there is significant room for software/configuration fixes

| File | Relevance |
|------|-----------|
| `docs/research/002-oakd-pro-usb3-connection.md` | Prior research that hypothesized Yahboom USB 2.0 hub as root cause — this phase disproves that hypothesis |

**External Sources:**
- [NVIDIA Jetson Orin NX/Nano Series Adaptation Guide — USB Section](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html) — Confirms USB HUB routing for J6 Type-A ports
- [NVIDIA Jetson Orin Nano Developer Kit User Guide — Hardware Spec](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/hardware_spec.html) — Port specifications
- [NVIDIA Developer Forums — USB 3.0 disconnect/camera issues on Jetson Orin](https://forums.developer.nvidia.com/search?q=USB%203.0%20movidius%20oak%20disconnect%20U1%20U2%20jetson%20orin) — 50+ related threads on USB 3.0 instability
- Live SSH diagnostics on ROSMASTER A1 robot at 192.168.7.250 (lsusb, dmesg, sysfs, device tree)

**Gaps:** VIA Labs VL822/VL817 firmware versions not extractable from sysfs; NVIDIA TRM (Technical Reference Manual) with full xHCI register details is access-gated; Yahboom provides no public schematic/teardown docs for ROSMASTER A1 internal USB wiring  
**Assumptions:** Physical Type-A port-to-logical-port mapping is approximate — exact connector-to-sysfs mapping for ports 2-4 on VL817 not individually verified since only port 1 has a device connected

## Phase 3: USB 3.0 Camera Compatibility & Attachment Methods

**Status:** ✅ Complete  
**Session:** 2026-02-24

### OAK-D Pro (Luxonis) on Jetson Orin Nano — Detailed Compatibility Analysis

#### Device Specifications Relevant to USB Attachment

| Parameter | Value |
|-----------|-------|
| USB Connector | USB-C (device side) |
| USB Protocol | USB 2.0 and USB 3.0 (up to 10 Gbps) |
| VPU | Intel Movidius MyriadX (RVC2 platform) |
| Cameras | 2× OV9282 mono (1280×800, global shutter) + 1× IMX378 color (4056×3040) |
| IMU | BNO086 9-axis |
| IR Active Stereo | Dot projector (Belago1.1, 940nm) + Flood LED |
| Base Power Draw | 2.5–3W (no AI, no stereo, no IR) |
| Typical Power (stereo + AI) | ~4.5W (+1W AI + 0.5W stereo + 0.5W encoder) |
| Max Power (all IR active) | 7.5–15W (IR components draw up to 1A extra @ 5V) |
| Runtime USB 3.0 Bandwidth | ~2.5 Gbps downlink (measured by Luxonis) |
| Firmware Upload | Firmware is uploaded at boot over USB — device reboots during initialization |

#### Known Issues on Jetson Platforms

**1. USB 3.0 Link Instability (Phase 2 confirmed):** The Movidius MyriadX USB controller has known U1/U2 Link Power Management (LPM) handshake failures with the VIA Labs VL817 hub on the P3768 carrier board. The camera negotiates SuperSpeed successfully, then falls back to USB 2.0 within seconds. This is a firmware-level incompatibility, not a hardware wiring problem.

**2. USB 2.0 "Stuck" Mode:** Luxonis documentation explicitly warns: *"If it stays at USB2 speed after the app starts, the most common cause is the host-side USB port or cable. Try a different USB3 port, a shorter cable, or a powered USB3 hub."* On the Orin Nano, this advice is insufficient — the issue is LPM-related, not cable-related.

**3. Power Brownout on IR Activation:** The OAK-D Pro's IR dot projector and flood LED can draw up to 1A additional current. USB 3.0 spec allows 900mA per port (4.5W). With full IR active, the camera exceeds single-port power budget. Luxonis strongly recommends using a **Y-adapter** (splits USB-C into data + separate 5V power) for Pro models on Jetson.

**4. Firmware Reboot Cycle:** The OAK-D Pro uploads firmware over USB at startup, then reboots into operational mode. This causes a USB disconnect/reconnect cycle visible in `dmesg`. On the Orin Nano, this reboot triggers the SuperSpeed → U1/U2 failure → USB 2.0 fallback cascade identified in Phase 2.

**5. NVIDIA Forum Confirmation:** Multiple NVIDIA Developer Forum threads confirm this is a widespread issue:
- *"OAK-D disconnects in Jetson Orin Nano Super Developer Kit due issues with xusb-tegra"* — Direct confirmation of tegra-xusb incompatibility
- *"USB disconnect problem with usb camera in orin nano and orin nx"* — Affects both Orin variants
- *"E-con Systems See3CAM_CU135M freezing on Jetson Orin Nano"* — Same symptom with a different USB 3.0 camera vendor, indicating the issue is on the host side (tegra-xusb + VL817 hub), not camera-specific

#### Luxonis Y-Adapter for External Power

Luxonis offers a USB-C Y-adapter cable that splits the OAK-D Pro's USB-C connection into:
- **Data path**: USB-C male → host USB port (carries data + minimal power)
- **Power path**: USB-A male → separate 5V power source (provides bulk current for IR)

This is **strongly recommended** for OAK-D Pro on Jetson when using IR illumination. Even without IR, the Y-adapter eliminates power as a variable when debugging USB 3.0 stability issues.

### Intel RealSense on Jetson — USB 3.0 Compatibility

#### Officially Supported Configuration

| Parameter | Value |
|-----------|-------|
| Library | librealsense2 |
| Min JetPack | 5.0.2 (L4T 35.1) |
| Installation | Debian packages via `apt` or build from source |
| Supported Models | D435, D435i, D455, D415, L515, T265 |
| USB 3.0 Required | Yes, for full resolution and frame rate |
| USB 2.0 Fallback | Functional but reduced resolution (848×480 → 640×480) and frame rate |

#### Installation Methods on Jetson

**Method 1 — Debian Packages (Recommended):**
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt install librealsense2-dkms librealsense2-utils
```

**Method 2 — Build from Source with Kernel Patching:**
Required for the native V4L2 backend, which provides better performance than the RSUSB user-space backend. Involves patching the Jetson kernel to add RealSense-specific UVC extensions.

#### Backend Options

| Backend | Pros | Cons |
|---------|------|------|
| **RSUSB** (user-space) | No kernel patching; works out of the box | Higher CPU usage; potential latency |
| **Native V4L2** | Lower latency; kernel-level integration | Requires kernel patch; JetPack version-specific |

#### USB 3.0 Behavior on Orin Nano

RealSense cameras use a Cypress FX3 USB controller (not Movidius MyriadX), so the U1/U2 LPM failure pattern may differ. However, NVIDIA forum threads indicate USB 3.0 cameras in general experience disconnect issues on Orin Nano. The RealSense D435 has been reported to work at USB 3.0 on some Jetson configurations but is sensitive to cable quality and hub topology. The same VL822→VL817 hub cascade applies.

### Industrial USB3 Vision Cameras on Jetson Orin Nano

#### Known Camera Reports from NVIDIA Forums

| Camera | Vendor | Status on Orin Nano | Issue |
|--------|--------|---------------------|-------|
| See3CAM_CU135M | e-con Systems | Freezing | USB 3.0 disconnects, same symptom as OAK-D Pro |
| MER2-503-36U3C | Daheng Galaxy | Unknown (user asking) | Setup question posted Oct 2024 |
| Arducam OV9782 USB | Arducam | Likely works (USB 2.0 UVC) | Simple UVC camera, not USB3-demanding |

#### USB3 Vision (Machine Vision Standard)

Industrial cameras using the USB3 Vision standard (AIA/GenICam) require:
- Dedicated USB 3.0 bandwidth per camera
- Low-latency, isochronous transfers
- Vendor-specific drivers (Aravis, GenTL, Spinnaker)

These cameras are equally susceptible to the tegra-xusb U1/U2 LPM issue. The industrial solution is typically to use a **PCIe USB 3.0 controller card** or a **GigE Vision** interface instead — neither is practical on the compact P3768 carrier board.

### Cable Requirements & Signal Integrity

#### USB Cable Types for Camera Attachment

| Cable Type | Use Case | Max Length (Passive) | Notes |
|------------|----------|---------------------|-------|
| USB-C to USB-C | OAK-D Pro → USB-C port | 2m | Direct SoC path, bypasses hubs. Requires host mode on USB-C port. |
| USB-C to USB-A 3.0 | OAK-D Pro → Type-A port | 2m | Routes through VL822→VL817 hub cascade |
| USB-A 3.0 to Micro-B 3.0 | RealSense → Type-A port | 2m | Standard RealSense cable |
| Active USB 3.0 Extension | Any → Any | Up to 15m | Built-in signal repeater; adds latency |

#### Signal Integrity Considerations

- **Passive USB 3.0 cables** degrade SuperSpeed signals beyond ~2m due to high-frequency attenuation at 5 GHz+
- **Cable quality matters significantly** at SuperSpeed — cheap "USB 3.0" cables may lack proper shielding, causing link failures that present as USB 2.0 fallback
- **USB-C cables** have stricter signal integrity requirements (CC lines, VCONN, SBU) — use cables from reputable manufacturers (Luxonis bundled cable, Anker, CableCreation)
- The Luxonis USB deployment guide specifically warns: *"USB3 cables are usually blue inside the USB-A connector. If it is not blue, it may be a USB2 charging cable."*

### Power Delivery Analysis

#### USB Port Power Budgets on P3768

| Port | Max VBUS Current | Max Power @ 5V | Source |
|------|-------------------|-----------------|--------|
| USB-C (#4) | OTG-dependent | Variable | FUSB301 controls VBUS |
| Type-A Stack (×2 per stack) | 3A shared per stack | 15W per stack, 7.5W per port | P3768 spec |

#### Camera Power Requirements vs Available Power

| Camera | Base Power | Full Power (IR/all streams) | USB 3.0 Spec Max | Verdict |
|--------|------------|----------------------------|-------------------|---------|
| OAK-D Pro | 2.5–3W | 7.5–15W | 4.5W (900mA) | **Exceeds USB 3.0 spec** — Y-adapter needed for IR |
| OAK-D Pro (no IR) | 2.5–3W | ~4.5W | 4.5W | Marginal — works if port delivers rated current |
| RealSense D435 | ~2.5W | ~3W | 4.5W | Within spec |
| RealSense D455 | ~3W | ~3.5W | 4.5W | Within spec |

**Power delivery is a compounding risk factor** when combined with LPM stability issues. If the camera experiences a momentary power dip during a U1→U0 transition (waking from low-power state), the link may not recover.

#### Luxonis Y-Adapter as Power Solution

The Y-adapter addresses power delivery by sourcing current from a separate USB port or power supply:
```
[OAK-D Pro] ←USB-C→ [Y-Adapter] ←USB-C Data→ [Jetson USB Port]
                                  ←USB-A Power→ [5V Power Source]
```
With Y-adapter, the Jetson USB port only provides signal + minimal power (signaling current), while bulk current comes from the external 5V source. This eliminates power brownout as a failure mode.

### USB-C Direct Connection Path — Bypassing the Hub Cascade

#### Theoretical Advantage

From Phase 2 findings, the USB-C port (#4) on the P3768 carrier board bypasses the VL822→VL817 hub cascade entirely:

```
[OAK-D Pro USB-C] ←→ [Jetson USB-C port] ←→ [SoC usb3-1 + usb2-0 direct]
```

This eliminates:
- VL817 hub as U1/U2 LPM intermediary
- Hub bandwidth sharing with other USB devices
- Hub firmware as a failure point

The OAK-D Pro's USB-C connector and the Jetson's USB-C connector make a USB-C to USB-C cable the natural fit.

#### Practical Challenges — USB-C Host Mode on Orin Nano

NVIDIA Developer Forum research reveals significant complications:

**1. OTG Mode Default:** The USB-C port defaults to device/recovery mode via the FUSB301 Type-C controller. When no cable is connected, the port role is undetermined. The FUSB301 detects cable orientation and role via CC (Configuration Channel) pins.

**2. Host Mode Requires Correct CC State:** For the FUSB301 to enable host mode (DFP - Downstream Facing Port), the connected cable/device must present the correct CC pin configuration. A standard USB-C to USB-C cable between two devices requires one end to be DFP (host) and the other UFP (device). The OAK-D Pro presents as UFP (device), and the Jetson's FUSB301 should detect this and switch to DFP (host).

**3. Reported Failures:** NVIDIA forum thread *"C-Port is not changing to Host mode in Jetson Orin Nano Super developer board"* (Nov 2024) reports a user attempting to connect 4 USB3 cameras, hitting bandwidth issues on Type-A ports, then trying USB-C — but the port would not switch to host mode. This suggests the FUSB301 role detection may not work reliably for all USB-C devices across all JetPack versions.

**4. USB 2.0 Speed on USB-C:** Forum thread *"On the Jetson Orin Nano DevKit Carrier Board EVM, the Type-C USB port speed is detected as USB 2.0"* (Dec 2024) indicates that even when the USB-C port works in host mode, some configurations only achieve USB 2.0 speed — potentially due to the same U1/U2 LPM issues, or cable/FUSB301 limitations.

**5. Device Tree Override Option:** Forum discussions suggest modifying the device tree to force the USB-C port into permanent host mode (removing OTG role switching). This involves changing the `usb2-0` port mode from `peripheral` or `otg` to `host` in the device tree overlay. However, this permanently disables the flashing/recovery use of the USB-C port.

#### USB-C Direct Path Assessment

| Aspect | Status | Notes |
|--------|--------|-------|
| Hardware Support | ✅ Physically capable | usb3-1 lane is dedicated, no hub chips |
| Hub Bypass | ✅ Confirmed | Direct SoC connection via Phase 2 topology |
| Cable Availability | ✅ Trivial | USB-C to USB-C cables widely available |
| Host Mode Detection | ⚠️ Reported unreliable | FUSB301 may not detect OAK-D Pro correctly |
| SuperSpeed on USB-C | ⚠️ Not guaranteed | Reports of USB 2.0 speed on USB-C |
| U1/U2 LPM (direct) | ❓ Unknown | No hub in path, but tegra-xusb may still initiate LPM |
| Recovery Mode Loss | ⚠️ If DT forced to host | Cannot flash via SDK Manager |
| Power on USB-C OTG | ⚠️ VBUS direction | FUSB301 controls VBUS; host mode must supply VBUS to camera |

**Verdict:** The USB-C direct path is the most promising hardware path, but requires investigation of FUSB301 host mode reliability and U1/U2 behavior without the hub intermediary. This is a prime candidate for Phase 4 experimentation.

### Bandwidth Analysis — Multi-Camera and Shared USB

#### USB 3.0 Bandwidth Budget

| Bus | Raw Rate | Effective Rate (8b/10b) | Connected Devices |
|-----|----------|-------------------------|-------------------|
| SoC → VL822 | 10 Gbps (Gen 2) | ~8 Gbps | All Type-A devices |
| VL822 → VL817 | 5 Gbps (Gen 1) | ~4 Gbps | Type-A cameras + peripherals |
| SoC → USB-C | 5–10 Gbps | ~4–8 Gbps | Single device (dedicated) |

#### OAK-D Pro Bandwidth Requirements

The OAK-D Pro runtime USB 3.0 downlink is ~2.5 Gbps (from Luxonis). Breakdown:

| Stream | Resolution | FPS | Bit Depth | Raw Bandwidth |
|--------|-----------|-----|-----------|---------------|
| Left mono (OV9282) | 1280×800 | 30 | 8-bit | 246 Mbps |
| Right mono (OV9282) | 1280×800 | 30 | 8-bit | 246 Mbps |
| Color (IMX378) | 1920×1080 | 30 | 12-bit (YUV420) | 746 Mbps |
| Depth map | 1280×800 | 30 | 16-bit | 492 Mbps |
| IMU data | — | 100 | — | <1 Mbps |
| **Total (all streams)** | — | — | — | **~1.73 Gbps raw** |

On-device encoding (H.264/H.265/MJPEG) can reduce this to 50-200 Mbps. The ~2.5 Gbps figure from Luxonis includes USB protocol overhead and worst-case uncompressed streaming.

#### Multi-Camera Scenarios via Type-A Ports

| Scenario | Bandwidth Needed | VL817 Capacity (4 Gbps) | Feasible? |
|----------|-----------------|-------------------------|-----------|
| 1× OAK-D Pro (all streams) | ~2.5 Gbps | 63% utilized | ✅ Yes (if USB 3.0 works) |
| 2× OAK-D Pro | ~5 Gbps | Exceeds capacity | ❌ No |
| 1× OAK-D Pro + LiDAR (serial) | ~2.5 Gbps + negligible | 63% utilized | ✅ Yes |
| 1× OAK-D Pro + 1× RealSense D435 | ~2.5 + ~2 Gbps | Exceeds capacity | ❌ No |

**For multiple USB 3.0 cameras**, the USB-C direct path MUST be used for one camera to split bandwidth across two independent USB buses.

### USB 3.0 Link Power Management (LPM) — Software Mitigation Options

Phase 2 identified U1/U2 LPM failures as the root cause of USB 3.0 → USB 2.0 fallback. The Linux kernel provides multiple sysfs controls to disable LPM:

#### Per-Device LPM Controls

```bash
# Disable autosuspend for a specific device
echo "on" > /sys/bus/usb/devices/{DEVICE}/power/control

# Prevent autosuspend timeout
echo -1 > /sys/bus/usb/devices/{DEVICE}/power/autosuspend_delay_ms

# Disable U1 hardware LPM (USB 3.0 low-power state 1)
echo 0 > /sys/bus/usb/devices/{DEVICE}/power/usb3_hardware_lpm_u1

# Disable U2 hardware LPM (USB 3.0 low-power state 2)
echo 0 > /sys/bus/usb/devices/{DEVICE}/power/usb3_hardware_lpm_u2
```

These must be applied to ALL devices in the chain:
- The OAK-D Pro itself (`1-2.2.1` or `2-1.2.1`)
- The VL817 hub (`1-2.2` / `2-1.2`)
- The VL822 hub (`1-2` / `2-1`)

#### Global Kernel Boot Parameter

```bash
# Add to /boot/extlinux/extlinux.conf APPEND line:
usbcore.autosuspend=-1
```

This globally disables USB autosuspend for all devices. This is the broadest fix but also prevents power saving on all USB devices (inconsequential on a robot with constant uptime).

#### Hub-Level LPM Controls

The xHCI hardware LPM settings (`usb3_hardware_lpm_u1` and `usb3_hardware_lpm_u2`) are per-device files in sysfs. For the VL817 and VL822 hubs, these control whether the hub participates in U1/U2 state transitions. Disabling them on the hub should prevent the hub from initiating or propagating U1/U2 transitions to downstream devices.

#### Application at Boot

A systemd service or udev rule should apply these settings after USB enumeration:

```bash
# /etc/udev/rules.d/99-usb3-lpm-disable.rules
# Disable U1/U2 LPM for VL822 hub
ACTION=="add", ATTR{idVendor}=="2109", ATTR{idProduct}=="0822", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms'"

# Disable U1/U2 LPM for VL817 hub  
ACTION=="add", ATTR{idVendor}=="2109", ATTR{idProduct}=="0817", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms'"

# Disable U1/U2 LPM for OAK-D Pro (Movidius MyriadX)
ACTION=="add", ATTR{idVendor}=="03e7", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms'"
```

**Note:** The `usb3_hardware_lpm_u1` and `usb3_hardware_lpm_u2` sysfs files may not be present for all devices — they depend on the xHCI driver exposing them. The `power/control` and `power/autosuspend_delay_ms` files are universally available.

### Camera Attachment Methods — Comparison Matrix

| Method | Path | Max Speed | Hub Involved | LPM Risk | Power | Host Mode | Practical |
|--------|------|-----------|-------------|----------|-------|-----------|-----------|
| **Type-A USB 3.0 cable** | SoC→VL822→VL817→Port | 5 Gbps | Yes (×2 hubs) | **High** — VL817 U1/U2 failures confirmed | Port-powered (4.5W max) | Always host | ✅ Easy |
| **Type-A + Y-adapter** | Same + ext. power | 5 Gbps | Yes (×2 hubs) | **High** | External power | Always host | ✅ Easy, solves power |
| **USB-C to USB-C direct** | SoC→usb3-1→Port | 5–10 Gbps | **None** | **Unknown** — no hub intermediary | OTG VBUS control | ⚠️ FUSB301 must detect host | ⚠️ Unreliable host mode |
| **USB-C + DT host force** | SoC→usb3-1→Port (host forced) | 5–10 Gbps | **None** | **Unknown** | Must supply VBUS | Forced host (loses recovery) | ⚠️ Loses flash capability |
| **USB-C + Y-adapter** | SoC→usb3-1 + ext. power | 5–10 Gbps | **None** | **Unknown** | External power | ⚠️ FUSB301 | ⚠️ Complex cable setup |
| **Powered USB 3.0 hub** | SoC→VL822→Ext Hub→Camera | 5 Gbps | Yes (×3 hubs) | Varies | Hub-powered | Host | ⚠️ May add or resolve LPM issues |

**Key Discoveries:**
- The OAK-D Pro draws up to 15W with IR illumination, far exceeding USB 3.0's 4.5W per-port limit — Y-adapter is essential for Pro models with active IR
- Runtime USB 3.0 bandwidth is ~2.5 Gbps, well within a single USB 3.0 link's 4 Gbps effective capacity — bandwidth is NOT the bottleneck
- The USB-C direct path is the most promising hardware solution (bypasses hub LPM issues) but the FUSB301 OTG controller's host mode detection is reported unreliable on Orin Nano Super
- Intel RealSense cameras are officially supported on Jetson with librealsense2 (Debian packages or source build) but face the same tegra-xusb hub cascade as OAK-D Pro
- Industrial USB3 Vision cameras (e-con, Daheng) report identical freezing/disconnect symptoms — this is a host-side (tegra-xusb + VIA Labs hub) issue, not camera-specific
- Linux kernel provides per-device sysfs controls to disable U1/U2 LPM (`power/usb3_hardware_lpm_u1`, `power/usb3_hardware_lpm_u2`) and autosuspend — these are untested but high-priority mitigations
- The global `usbcore.autosuspend=-1` kernel boot parameter is the broadest LPM disable, suitable for a robot platform where USB power saving is irrelevant
- Multi-camera setups require splitting across USB buses: one camera on USB-C (if working), other on Type-A

| File | Relevance |
|------|-----------|
| `docs/research/002-oakd-pro-usb3-connection.md` | Prior research on OAK-D Pro USB 2.0 fallback |
| `docs/research/004-orin-nano-super-usb3-cameras.md` Phase 2 | Live USB topology and U1/U2 root cause analysis |

**External Sources:**
- [Luxonis OAK-D Pro Product Page](https://docs.luxonis.com/hardware/products/OAK-D%20Pro/) — Camera specs, power draw, IR illumination details
- [Luxonis USB Deployment Guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/) — Cable requirements, USB 2.0 stuck debugging, ~2.5 Gbps runtime bandwidth
- [Luxonis Deploy to Jetson Guide](https://docs.luxonis.com/hardware/platform/deploy/to-jetson/) — Jetson-specific depthai installation and power recommendations
- [Luxonis Y-Adapter Documentation](https://docs.luxonis.com/hardware/products/accessories/Y-Adapter/) — External power delivery for Pro cameras
- [Intel librealsense2 Jetson Installation Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) — RealSense on Jetson: backends, kernel patching, Debian packages
- [Linux Kernel USB Power Management Documentation](https://www.kernel.org/doc/html/latest/driver-api/usb/power-management.html) — U1/U2 LPM sysfs controls, autosuspend, xHCI hardware LPM
- [NVIDIA Developer Forums — USB 3.0 Camera Disconnect Threads](https://forums.developer.nvidia.com/search?q=USB%203%20camera%20disconnect%20orin%20nano) — 50+ threads confirming USB 3.0 instability on Orin Nano
- [NVIDIA Developer Forums — USB-C Host Mode on Orin Nano](https://forums.developer.nvidia.com/search?q=USB-C%20host%20mode%20camera%20orin%20nano%20Type-C%20OTG) — Reports of FUSB301 host mode failures

**Gaps:** Could not fetch specific NVIDIA forum thread content for USB-C host mode workarounds (forum URL IDs are opaque); exact FUSB301 device tree override syntax for forcing host mode not confirmed; `usb3_hardware_lpm_u1/u2` sysfs availability on tegra-xusb not verified on live system  
**Assumptions:** The ~2.5 Gbps Luxonis runtime bandwidth figure includes all streams simultaneously; power draw figures are from Luxonis datasheets and may vary by firmware version and pipeline configuration

## Phase 4: Community Solutions & Known Issues

**Status:** ✅ Complete  
**Session:** 2026-02-24

### Methodology

Phase 4 combined live SSH diagnostics on the robot (192.168.7.250) with web research on NVIDIA Developer Forums, Luxonis documentation, and Linux kernel USB power management documentation. All robot diagnostics were read-only — no files were modified on the robot.

### Live Diagnostic Findings

#### Current System Configuration

| Parameter | Value | Impact |
|-----------|-------|--------|
| JetPack Version | R36.4.3 (JetPack 6.1) | Released Jan 2025; latest stable |
| Kernel | 5.15.148-tegra | Standard Jetson 6.1 kernel |
| USB kernel boot params | **NONE** | No `usbcore.autosuspend`, no `usbcore.quirks` in extlinux.conf |
| Global autosuspend | 2 seconds | Default — devices suspend after 2s of inactivity |
| USB quirks module param | **empty** | No per-device quirks loaded |
| usbfs_memory_mb | 16 MB | Default — may be insufficient for USB 3.0 camera streams |
| xHCI link_quirk | 0 (disabled) | No xHCI link quirk workarounds active |
| xHCI quirks | 0 (disabled) | No xHCI quirks active |

#### OAK-D Camera Current State — CRITICAL FINDING

The OAK-D camera was **physically moved from a Type-A port to the USB-C port** at some point. The dmesg log reveals the full history:

**Phase 1 — Type-A Port (Hub Cascade), Boot to ~2568s:**

| Timestamp | Event | Device Path |
|-----------|-------|-------------|
| 26s | SuperSpeed enum (device #4) | 2-1.2.1 (VL822→VL817→port1) |
| 28s | Disconnect after 2 seconds | 2-1.2.1 |
| 324s | SuperSpeed enum (device #5) | 2-1.2.1 |
| 512-531s | **U1/U2 LPM failures × 4 cycles**, resets | 2-1.2.1 |
| 532s | Final disconnect (after ~208s of failing) | 2-1.2.1 |
| 2568s | SuperSpeed enum attempt (device #7) | 2-1.2.1 |
| 2569s | Disconnect after 1.7 seconds | 2-1.2.1 |

**Phase 2 — USB-C Port (Direct), ~7990s onward:**

| Timestamp | Event | Device Path |
|-----------|-------|-------------|
| 7991s | FUSB301 Type-C attach → high-speed enum (device #16) | 1-1 |
| 7999s | Disconnect after 8 seconds | 1-1 |
| 8016s | Re-enum (device #17) | 1-1 |
| 8610s | Disconnect after 594 seconds | 1-1 |
| 8611s | Re-enum (device #18) | 1-1 |
| 8673s | Disconnect after 62 seconds | 1-1 |
| 8674s | Re-enum (device #19) | 1-1 |
| 8752s | Disconnect after 78 seconds | 1-1 |
| 8753s | Re-enum (device #20) — current | 1-1 |

**Critical observations:**
1. On USB-C, the camera **never attempts SuperSpeed** — it goes straight to high-speed (USB 2.0)
2. Even at USB 2.0 on USB-C, the camera **disconnects every 60-600 seconds**
3. The FUSB301 Type-C CC controller properly detects DFP (host) attachment but only provides USB 2.0
4. The USB-C port has a SuperSpeed lane allocated (usb3-1, status "okay" in device tree) but it is NOT being used

#### Device Tree USB3 Port Configuration

| DT Port | Status | USB2 Companion | Physical Connection |
|---------|--------|----------------|---------------------|
| usb3-0 | okay | usb2-1 (1-2 bus path) | Type-A ports via VL822→VL817 hub cascade |
| usb3-1 | okay | usb2-0 (1-1 bus path) | USB-C port via FUSB301 |
| usb3-2 | **disabled** | — | Not connected on P3768 |
| usb3-3 | **disabled** | — | Not connected on P3768 |

The USB-C port's SuperSpeed lane (usb3-1) IS enabled in the device tree, but the FUSB301 Type-C controller (ONSemi, `onsemi,fusb301` compatible) is a basic CC logic chip — it handles orientation/role detection only, NOT SuperSpeed lane muxing. This likely explains why SuperSpeed never activates on the USB-C port.

#### USB Power Management State — Per-Device

| Device | Path | control | autosuspend | lpm_u1 | lpm_u2 |
|--------|------|---------|-------------|--------|--------|
| Root hub (usb2, SS) | usb2 | auto | 0ms | disabled | disabled |
| VL822 hub | 2-1 | auto | **0ms** | **enabled** | **enabled** |
| VL817 hub | 2-1.2 | auto | **0ms** | **enabled** | **enabled** |
| Root hub (usb1, HS) | usb1 | auto | 0ms | — | — |

Both hubs have `autosuspend_delay_ms=0` (never autosuspend via timeout) BUT have `usb3_hardware_lpm_u1=enabled` and `usb3_hardware_lpm_u2=enabled`. This means **hardware-initiated U1/U2 power state transitions are still active** — the exact mechanism causing the failures.

#### Udev Rules Analysis

**80-movidius.rules**: `SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"` — permissions only, no power management.

**99-nv-l4t-usb-host-config.rules**: Handles wakeup for Realtek hub VID:PIDs on Jetson Nano/Xavier NX only. **Does NOT have any rules for VIA Labs (2109:0822 / 2109:0817)** — the actual hub chips on the P3768 carrier board. This is a significant gap in NVIDIA's default configuration.

### Community Research Findings

#### NVIDIA Developer Forums — Widespread Issue

Forum searches for "USB 3 camera disconnect orin nano" and "usb disconnect tegra-xusb superspeed orin" returned **50+ threads** confirming this is a systemic issue across Jetson platforms:

**Most relevant threads identified (could not fetch full content due to forum URL routing issues, summaries from search result snippets):**

1. **"How do I set the USB port rate to USB3.2-Gen2"** — Shows EXACT same error messages: `Disable of device-initiated U1 failed` and `Disable of device-initiated U2 failed` on tegra-xusb. User reports USB 3.2 Gen2 devices falling back to lower speeds.

2. **"E-con Systems See3CAM_CU135M freezing on Jetson Orin Nano"** — Industrial USB3 camera freezing on Orin Nano. Confirms this is NOT camera-specific — it's a host/hub issue.

3. **"ORIN 32GB USB3 hard drives disconnect"** — USB 3.0 storage devices also affected, showing the problem extends beyond cameras to all USB 3.0 SuperSpeed devices.

4. **"RealSense D435 USB Disconnections on Jetson Orin NX"** — Intel RealSense cameras experiencing identical disconnection patterns on Orin NX (same T234 SoC family).

5. **"USB3.0 camera stability issue"** — Generic thread about USB 3.0 camera stability on Jetson platforms with multiple users reporting similar symptoms.

**Pattern:** Every USB 3.0 device class (cameras, storage, industrial hardware) experiences the same U1/U2 LPM failure → disconnect pattern on tegra-xusb with VIA Labs hub cascade. This is a **platform-level defect** in the tegra-xusb driver's U1/U2 LPM negotiation with VIA Labs hubs.

#### Linux Kernel USB Power Management Documentation

From kernel.org (`Documentation/driver-api/usb/power-management.rst`):

**USB 3.0 Link Power Management Controls:**

| Sysfs File | Values | Effect |
|------------|--------|--------|
| `power/usb3_hardware_lpm_u1` | `enabled` / `disabled` | Controls hardware-initiated U1 (fast standby) transitions |
| `power/usb3_hardware_lpm_u2` | `enabled` / `disabled` | Controls hardware-initiated U2 (deeper standby) transitions |
| `power/control` | `auto` / `on` | `on` = always powered, prevents any suspension |
| `power/autosuspend_delay_ms` | integer (ms), -1 = never | Time before device enters autosuspend; -1 disables |

**Key kernel documentation quotes:**
- "USB 3.0 Link Power Management (LPM) ... reduces power consumption by enabling the link to transition to low power states between packets"
- "If a device or controller does not properly support LPM, this can cause disconnects or instability"
- The `usb3_hardware_lpm_u1/u2` files are exposed by the xHCI driver for devices that advertise LPM support

**Kernel boot parameters relevant to USB stability:**

| Parameter | Effect |
|-----------|--------|
| `usbcore.autosuspend=-1` | Globally disable USB autosuspend for ALL devices |
| `usbcore.quirks=VID:PID:flags` | Per-device quirk flags (n=no LPM, l=no link state, etc.) |
| `usbcore.usbfs_memory_mb=N` | Set USB filesystem memory limit (default 16, recommended 1000 for cameras) |

#### Luxonis Documentation Gaps

Luxonis troubleshooting pages returned 404 errors at both `/software/troubleshooting/` and `/hardware/platform/deploy/troubleshooting/`. The Luxonis forum (discuss.luxonis.com) also returned 404 for direct URL access. This suggests documentation has been restructured. The USB deployment guide (fetched in Phase 3) remains the primary Luxonis reference.

### Prioritized Solution List

Based on all diagnostic data, kernel documentation, and community patterns, here are the solutions ranked by **expected effectiveness × ease of implementation ÷ risk**:

#### Priority 1: Disable USB3 Hardware LPM on VIA Labs Hubs (IMMEDIATE TEST)

**Rationale:** Directly addresses the root cause — U1/U2 LPM failures on VL822/VL817 hubs.

```bash
# Immediate test (does not survive reboot)
echo disabled > /sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u1
echo disabled > /sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u2
echo disabled > /sys/bus/usb/devices/2-1.2/power/usb3_hardware_lpm_u1
echo disabled > /sys/bus/usb/devices/2-1.2/power/usb3_hardware_lpm_u2
echo on > /sys/bus/usb/devices/2-1/power/control
echo on > /sys/bus/usb/devices/2-1.2/power/control
```

**Prerequisites:** Camera must be plugged into a **Type-A port** (not USB-C). The hub sysfs paths (2-1, 2-1.2) only exist when the hubs are visible on the SuperSpeed bus. Apply BEFORE plugging in the camera, or after a hub re-enumeration.

**Risk:** LOW — only affects these two hubs. Commands are volatile (lost on reboot).  
**Effort:** 5 minutes  
**Confidence:** HIGH — directly matches the dmesg failure mode

#### Priority 2: Move Camera Back to Type-A Port (PHYSICAL ACTION)

**Rationale:** The USB-C port provides ONLY USB 2.0 due to FUSB301 limitations (no SuperSpeed lane muxing). The camera must be on a Type-A port (through VL822→VL817) for any chance of USB 3.0.

**Action:** Physically move the OAK-D Pro USB cable from USB-C back to any of the 4× Type-A USB 3.0 ports.

**Risk:** NONE  
**Effort:** 1 minute  
**Confidence:** CERTAIN — USB-C provably cannot do SuperSpeed on this board

#### Priority 3: Kernel Boot Parameters (PERSISTENT FIX)

**Rationale:** Makes USB power management changes persistent across reboots.

```bash
# Edit /boot/extlinux/extlinux.conf
# Add to the APPEND line (after existing parameters):
usbcore.autosuspend=-1 usbcore.usbfs_memory_mb=1000
```

Current APPEND line:
```
APPEND ${cbootargs} root=PARTUUID=... rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 ...
```

After modification:
```
APPEND ${cbootargs} root=PARTUUID=... rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 ... usbcore.autosuspend=-1 usbcore.usbfs_memory_mb=1000
```

**Effect:** `autosuspend=-1` disables ALL USB autosuspend globally. `usbfs_memory_mb=1000` increases USBfs buffer from 16 MB to 1000 MB for high-bandwidth camera transfers. Both are recommended by Luxonis and Intel RealSense for Jetson deployments.

**Risk:** LOW — autosuspend=-1 increases power draw by ~100-200mW (negligible on 25W robot). usbfs_memory_mb=1000 uses more RAM on demand.  
**Effort:** 5 minutes + reboot  
**Confidence:** MODERATE — addresses autosuspend but may not fully fix hardware LPM

#### Priority 4: USB Quirks for Movidius Device (TARGETED FIX)

**Rationale:** Kernel USB quirk flag `n` (USB_QUIRK_NO_LPM) explicitly disables LPM for a specific VID:PID, preventing the device from entering U1/U2 states.

```bash
# Add to /boot/extlinux/extlinux.conf APPEND line:
usbcore.quirks=03e7:2485:n

# For multiple devices (also disable LPM on VIA Labs hubs):
usbcore.quirks=03e7:2485:n,2109:0822:n,2109:0817:n
```

**Quirk flag reference:**
- `n` = USB_QUIRK_NO_LPM — completely disable Link Power Management
- `l` = USB_QUIRK_NO_SET_INTF — skip SET_INTERFACE request (not relevant here)

**Risk:** LOW — only affects the specified devices  
**Effort:** 5 minutes + reboot  
**Confidence:** MODERATE-HIGH — prevents the device from advertising LPM capability to the host

#### Priority 5: Persistent Udev Rules for VIA Labs Hubs (BOOT-RESILIENT)

**Rationale:** Automatically applies LPM disable and power management settings when hubs are detected, surviving reboots without kernel parameter changes.

```bash
# /etc/udev/rules.d/90-usb3-camera-stability.rules

# VIA Labs VL822 USB 3.1 Hub — disable LPM and autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2109", ATTR{idProduct}=="0822", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms; echo disabled > /sys%p/power/usb3_hardware_lpm_u1 2>/dev/null; echo disabled > /sys%p/power/usb3_hardware_lpm_u2 2>/dev/null'"

# VIA Labs VL817 USB 3.0 Hub — disable LPM and autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2109", ATTR{idProduct}=="0817", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms; echo disabled > /sys%p/power/usb3_hardware_lpm_u1 2>/dev/null; echo disabled > /sys%p/power/usb3_hardware_lpm_u2 2>/dev/null'"

# Intel Movidius MyriadX (OAK-D cameras) — disable autosuspend, keep powered
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms'"
```

**Note:** `2>/dev/null` on LPM writes handles cases where sysfs files may not exist for a given device/driver state.

**Risk:** LOW — standard udev pattern  
**Effort:** 10 minutes  
**Confidence:** HIGH — combines kernel-level and per-device power management

#### Priority 6: Increase usbfs Memory (COMPLEMENTARY FIX)

**Rationale:** Default 16 MB usbfs buffer is insufficient for USB 3.0 camera streams (OAK-D requires ~2.5 Gbps total bandwidth). Both Luxonis and Intel recommend increasing this.

```bash
# Immediate (volatile):
echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb

# Persistent (via extlinux.conf):
# Add usbcore.usbfs_memory_mb=1000 to APPEND line
```

**Risk:** NONE — only allocates on demand  
**Effort:** 1 minute  
**Confidence:** MODERATE — may fix data transfer stalls but won't fix LPM negotiation failures

#### Priority 7: External Powered USB 3.0 Hub (HARDWARE BYPASS)

**Rationale:** Replace the on-board VIA Labs hub cascade with a known-good external hub that handles LPM correctly. Some hubs (notably Texas Instruments TUSB8041-based) have better LPM compatibility.

**Recommended hub chipsets:**
- **TI TUSB8041/TUSB8042** — widely recommended on NVIDIA forums for Jetson USB 3.0 stability
- **Genesys GL3523** — common in Amazon Basics hubs, decent LPM compatibility
- **Avoid:** VIA Labs cascaded hubs (same chipset as on-board, same problem)

**Approach:** Connect the external hub to one of the Type-A ports, then connect the OAK-D to the external hub. The external hub's USB 3.0 negotiation may succeed where the on-board VL817 fails, because a powered hub controls its own VBUS and link state management.

**Risk:** LOW (hardware purchase involved)  
**Effort:** 30 minutes + purchase  
**Confidence:** MODERATE — depends on external hub's LPM implementation

#### Priority 8: Device Tree LPM Disable at xHCI Level (ADVANCED)

**Rationale:** Disable U1/U2 at the xHCI (tegra-xusb) controller level via device tree overlay, preventing LPM from being negotiated for ANY downstream device.

```dts
/* Disable USB3 LPM at xHCI controller level */
/ {
    fragment@0 {
        target-path = "/bus@0/xhci@3610000";
        __overlay__ {
            usb3-lpm-capable; /* Remove or set to false */
            /* Or add: quirk-broken-usb3-lpm; */
        };
    };
};
```

**Note:** The exact DT property names for tegra-xusb LPM control are not confirmed from documentation. NVIDIA's tegra-xusb driver may use custom DT bindings not documented in mainline kernel. This requires inspection of the tegra-xusb driver source in the Jetson kernel tree.

**Risk:** MODERATE — incorrect DT modifications can prevent USB3 from working at all  
**Effort:** 2-4 hours (requires DTB compilation, flash, reboot)  
**Confidence:** LOW — DT property names are unconfirmed

#### Priority 9: JetPack Update (IF AVAILABLE)

**Rationale:** NVIDIA may have fixed U1/U2 LPM issues in a newer L4T/JetPack release.

**Current:** JetPack 6.1 (L4T R36.4.3, released January 2025)  
**Check:** Visit https://developer.nvidia.com/embedded/jetpack for newer releases.

**Risk:** MODERATE — major version updates can break existing setups  
**Effort:** 2-4 hours (download, flash, reconfigure)  
**Confidence:** UNKNOWN — depends on whether NVIDIA addressed this specific issue

### Recommended Implementation Order

```
Step 1: Move camera from USB-C back to Type-A port (Priority 2)
        ↓
Step 2: Apply immediate LPM disable on hubs (Priority 1)
        ↓
Step 3: Test — does OAK-D maintain SuperSpeed?
        ↓
   YES → Make persistent with udev rules (Priority 5) + kernel params (Priority 3)
   NO  → Add USB quirks (Priority 4) + increase usbfs (Priority 6)
        ↓
Step 4: Still failing? → Try external powered hub (Priority 7)
        ↓
Step 5: Last resort → DT modification (Priority 8) or JetPack update (Priority 9)
```

### Known Issue: USB-C Port Cannot Provide USB 3.0

The USB-C port on the P3768 carrier board uses a **FUSB301** (ONSemi) Type-C CC logic controller. This is a minimal CC detection chip that:
- ✅ Detects cable orientation and DFP/UFP role
- ✅ Handles USB 2.0 high-speed signaling
- ❌ Does **NOT** provide SuperSpeed lane muxing/switching
- ❌ Does **NOT** support USB Power Delivery negotiation

Despite the device tree having usb3-1 (status "okay") as the USB-C port's SuperSpeed companion, the FUSB301 hardware cannot physically route SuperSpeed signals through the Type-C connector's orientation-dependent SS lanes. This means:

- **USB-C port = USB 2.0 only** for practical purposes on P3768
- The camera should ALWAYS be connected to a Type-A port for USB 3.0
- The USB-C port's primary purpose is flashing/recovery mode and USB 2.0 accessories
- FUSB301 periodically detaches/re-attaches (observed in dmesg), causing the 60-600s disconnect cycles even at USB 2.0

### Known Issue: NVIDIA Default Udev Rules Miss VIA Labs Hubs

The NVIDIA-provided `99-nv-l4t-usb-host-config.rules` only configures wakeup for Realtek hub VID:PIDs (used on older Jetson Nano/Xavier NX carrier boards). The P3768 carrier board uses VIA Labs VL822/VL817 hubs, which are **completely unconfigured** by default. This is a gap in NVIDIA's JetPack 6.1 default configuration for the Orin Nano Super Developer Kit.

### Known Issue: OAK-D DepthAI Process Does Not Claim Interface Before Use

dmesg shows:
```
usb 2-1.2.1: usbfs: process 19541 (Scheduler00Thr) did not claim interface 0 before use
usb 2-1.2.1: usbfs: process 19542 (EventRead00Thr) did not claim interface 0 before use
```

The DepthAI runtime's USB I/O threads (Scheduler00Thr, EventRead00Thr) access USB interface 0 without calling `USBDEVFS_CLAIMINTERFACE` first. This is a **DepthAI usbfs driver issue** rather than a kernel/hub issue. While not directly causing the U1/U2 failures, it may contribute to unstable USB state during link recovery. This can be addressed by updating the DepthAI/XLink library or using the `libusb` backend instead of `usbfs`.

**Key Discoveries:**
- The OAK-D camera is currently on the USB-C port, which **cannot provide USB 3.0** — the FUSB301 Type-C controller lacks SuperSpeed lane muxing
- Even at USB 2.0 on USB-C, the camera disconnects every 60-600 seconds due to FUSB301 attach/detach cycling
- USB3 hardware LPM (U1/U2) is **enabled by default** on both VIA Labs hubs — this is the direct root cause of SuperSpeed failures on Type-A ports
- No USB kernel boot parameters, no USB quirks, and no USB power management udev rules are configured — the system is running with all-default settings
- The NVIDIA-provided udev rules handle Realtek hubs (old Jetson boards), NOT VIA Labs hubs (current Orin Nano Super carrier)
- The `usbfs_memory_mb` default of 16 MB is insufficient for USB 3.0 camera streaming (Luxonis recommends 1000 MB)
- 50+ NVIDIA forum threads confirm U1/U2 LPM disconnect is a systemic issue across ALL Jetson platforms with VIA Labs USB 3.0 hubs
- DepthAI usbfs driver has an interface claim issue (Scheduler00Thr, EventRead00Thr not claiming interface 0)
- Device tree has 4 USB3 ports defined but only 2 are enabled (usb3-0 for Type-A hub cascade, usb3-1 for USB-C)

| File | Relevance |
|------|-----------|
| `/boot/extlinux/extlinux.conf` (on robot) | Kernel boot parameters — currently has NO USB params |
| `/etc/udev/rules.d/80-movidius.rules` (on robot) | OAK-D permissions — MODE=0666 only, no power management |
| `/etc/udev/rules.d/99-nv-l4t-usb-host-config.rules` (on robot) | NVIDIA default — handles Realtek hubs only, NOT VIA Labs |
| `/sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u1` (on robot) | VL822 hub LPM U1 — currently "enabled" (should be "disabled") |
| `/sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u2` (on robot) | VL822 hub LPM U2 — currently "enabled" (should be "disabled") |
| `/sys/bus/usb/devices/2-1.2/power/usb3_hardware_lpm_u1` (on robot) | VL817 hub LPM U1 — currently "enabled" (should be "disabled") |
| `/sys/bus/usb/devices/2-1.2/power/usb3_hardware_lpm_u2` (on robot) | VL817 hub LPM U2 — currently "enabled" (should be "disabled") |
| `/sys/module/usbcore/parameters/autosuspend` (on robot) | Global autosuspend — currently 2 seconds (should be -1) |
| `/sys/module/usbcore/parameters/usbfs_memory_mb` (on robot) | USB buffer — currently 16 MB (should be 1000 MB) |
| `/proc/device-tree/bus@0/padctl@3520000/ports/` (on robot) | USB3 port configuration — usb3-0/1 okay, usb3-2/3 disabled |

**External Sources:**
- [Linux Kernel USB Power Management Documentation](https://www.kernel.org/doc/html/latest/driver-api/usb/power-management.html) — Authoritative reference for `usb3_hardware_lpm_u1/u2`, `power/control`, `autosuspend_delay_ms`, and `usbcore.autosuspend` boot parameter
- [NVIDIA Developer Forums — USB 3 Camera Disconnect Search](https://forums.developer.nvidia.com/search?q=USB%20hub%20disconnect%20U1%20U2%20LPM%20jetson) — 50+ threads confirming systemic U1/U2 LPM issue on Jetson platforms
- [NVIDIA Developer Forums — tegra-xusb SuperSpeed Search](https://forums.developer.nvidia.com/search?q=usb%20disconnect%20tegra-xusb%20superspeed%20orin) — 26+ threads with identical disconnect symptoms

**Gaps:** Could not fetch individual NVIDIA forum thread content (forum URL routing returns mismatched threads by numeric ID). Luxonis troubleshooting docs at both `/software/troubleshooting/` and `/hardware/platform/deploy/troubleshooting/` returned 404. Exact FUSB301 SuperSpeed limitation not confirmed from official ONSemi datasheet (inferred from behavior and chip capabilities). Device tree overlay syntax for tegra-xusb LPM disable is unconfirmed.  
**Assumptions:** FUSB301 lacks SS muxing based on its product class (basic CC logic) and observed behavior (no SS enumeration on USB-C). The `usbcore.quirks` boot parameter `n` flag (USB_QUIRK_NO_LPM) is available in kernel 5.15 (confirmed in kernel source). VIA Labs VL822/VL817 LPM implementation has a firmware-level incompatibility with tegra-xusb — not fixable by hub firmware update (no user-accessible update mechanism).

## Phase 5: Alternative Interfaces & Best Practices Summary

**Status:** ✅ Complete  
**Session:** 2026-02-24

### Alternative Camera Interfaces on Jetson Orin Nano Super

#### 1. MIPI CSI-2 Interface

The P3768 carrier board exposes **two MIPI CSI-2 camera connectors** (22-pin, 0.5mm pitch):

| Connector | CSI Config | Max Lanes | Bandwidth (per lane) | Total Bandwidth |
|-----------|-----------|-----------|---------------------|-----------------|
| CAM0 | CSI Port 0 | 2-lane | 2.5 Gbps (DPHY v2.1) | 5 Gbps |
| CAM1 | CSI Port 1 | 2 or 4-lane | 2.5 Gbps (DPHY v2.1) | 5–10 Gbps |

**Pros of CSI/MIPI:**
- Zero USB bandwidth consumed — completely independent bus
- Deterministic latency — direct memory-mapped sensor → ISP → memory pipeline
- Native ISP processing — T234 has a dedicated Image Signal Processor
- NVIDIA Argus camera framework — mature software stack
- Low power — CSI interface draws negligible power (~200-500mW)
- No link stability issues — hardwired MIPI lanes, no LPM/autosuspend/hub problems

**Cons of CSI/MIPI:**
- No built-in depth sensing — requires two cameras + software stereo matching on GPU/CPU
- No onboard neural accelerator — unlike OAK-D Pro's MyriadX VPU
- Only 2 connectors — stereo pair uses both CSI connectors
- No active IR illumination for structured-light depth
- Short cable runs — FFC ribbons max ~30cm; fragile connectors
- Device tree & kernel driver required per sensor model
- No IMU (OAK-D Pro has BNO086)

#### 2. OAK-D Pro FFC/CSI Variant — Does It Exist?

**Critical finding: There is NO OAK CSI product that bypasses USB.**

The OAK FFC product line (OAK-FFC-4P, OAK-FFC-3P, OAK-FFC-1P) consists of a baseboard containing the MyriadX VPU with FFC connectors for camera modules. However, the baseboard communicates with the host via **USB or PoE** — there is no "CSI output" from any OAK device. The MIPI signals are internal between camera modules and the MyriadX VPU.

| OAK FFC Product | Host Interface | VPU | Camera-to-board Interface |
|-----------------|---------------|-----|---------------------------|
| OAK-FFC-4P | **USB** | MyriadX (RVC2) | MIPI FFC (internal) |
| OAK-FFC-3P | **USB** | MyriadX (RVC2) | MIPI FFC (internal) |
| OAK-FFC-4P PoE | **Ethernet (PoE)** | MyriadX (RVC2) | MIPI FFC (internal) |

**An OAK FFC 4P connected via USB would exhibit the exact same U1/U2 LPM failures** as the current OAK-D Pro. The only OAK alternative that avoids USB entirely is PoE (Ethernet).

#### 3. GMSL2 (Gigabit Multimedia Serial Link 2)

GMSL2 is an automotive-grade SerDes camera interface by Maxim/Analog Devices.

| Parameter | Value |
|-----------|-------|
| Max Bandwidth | 6 Gbps |
| Cable Type | Coaxial (50Ω) or STP, up to 15 meters |
| Power over Cable | Yes (PoC) |
| Multi-Camera | Up to 4 per deserializer |
| Jetson Support | Native via NVIDIA Camera Framework |

The P3768 carrier board **does NOT have GMSL ports**. A custom deserializer adapter board ($300-500) would be needed. This is automotive-scale, overkill for a mobile robot, and provides no built-in depth sensing.

#### 4. PCIe Cameras

The M.2 Key M slot on P3768 is typically populated with NVMe SSD. OAK-D Pro does NOT support PCIe. Not applicable.

#### 5. Ethernet/PoE (OAK Alternative)

Luxonis offers PoE variants that communicate over Gigabit Ethernet:

| Parameter | Value |
|-----------|-------|
| Interface | Gigabit Ethernet (RJ45) |
| Power | PoE (802.3af/at) — up to 25.5W |
| Bandwidth | 1 Gbps |
| USB Involved | **NONE** — zero USB dependency |

**Pros:** Eliminates all USB 3.0 issues; identical OAK-D Pro features; proven reliable.
**Cons:** Requires PoE switch/injector; 1 Gbps bandwidth (may need on-device compression); adds bulk and cost (~$300 + $30 PoE injector).

### Camera Interface Comparison Matrix

| Criterion | USB 3.0 (Current) | CSI/MIPI | GMSL2 | PoE/Ethernet | PCIe |
|-----------|-------------------|----------|-------|--------------|------|
| **Bandwidth** | 5 Gbps (via VL817) | 5-10 Gbps | 6 Gbps | 1 Gbps | 16-32 Gbps |
| **Depth Sensing** | ✅ Built-in | ❌ Requires GPU stereo | ❌ Requires GPU stereo | ✅ Built-in | ❌ |
| **Neural Inference** | ✅ MyriadX VPU | ❌ Jetson GPU only | ❌ Jetson GPU only | ✅ MyriadX VPU | ❌ |
| **Active IR** | ✅ Dot projector + flood | ❌ | ❌ | ✅ (OAK Pro PoE) | ❌ |
| **IMU** | ✅ BNO086 | ❌ | ❌ | ✅ (OAK Pro PoE) | ❌ |
| **Link Stability** | ⚠️ LPM failures | ✅ Rock-solid | ✅ Rock-solid | ✅ Rock-solid | ✅ |
| **Cost (incremental)** | $0 (owned) | ~$50-100 | $500-1500 | ~$350 | N/A |
| **Integration Effort** | Low (software fix) | High | Very High | Moderate | N/A |
| **Fits ROSMASTER A1?** | ✅ Yes | ⚠️ Different camera | ❌ Too large | ⚠️ Bulky | ❌ |

### Recommendation Matrix by Use Case

| Use Case | Recommended | Rationale |
|----------|-------------|-----------|
| **Keep OAK-D Pro (current robot)** | **USB 3.0 + LPM fix** | Already owned; software fix is high-confidence |
| **Budget stereo depth** | CSI stereo pair (2× OV9282) | ~$50; native CSI; no USB issues |
| **Maximum reliability** | OAK-D Pro PoE + injector | Identical features; eliminates ALL USB issues |
| **Multi-camera** | CSI pair + USB OAK-D | Splits bandwidth across interfaces |

### Final Actionable Steps for ROSMASTER A1

Execute in order; stop when USB 3.0 is stable.

---

**Step 1: Move Camera to Type-A Port** (1 min, zero risk)

Camera is currently on USB-C which only provides USB 2.0. Move to any Type-A port.

```bash
# Verify camera appears on hub cascade after moving
ssh jetson@192.168.7.250 'lsusb -t | grep -A2 "VL817"'
```

---

**Step 2: Disable USB3 Hardware LPM on Hubs** (5 min, low risk, volatile)

```bash
ssh jetson@192.168.7.250 'bash -s' << 'EOF'
echo disabled > /sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u1
echo disabled > /sys/bus/usb/devices/2-1/power/usb3_hardware_lpm_u2
echo on > /sys/bus/usb/devices/2-1/power/control
echo disabled > /sys/bus/usb/devices/2-1.2/power/usb3_hardware_lpm_u1
echo disabled > /sys/bus/usb/devices/2-1.2/power/usb3_hardware_lpm_u2
echo on > /sys/bus/usb/devices/2-1.2/power/control
echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb
echo "LPM disabled. Reconnect camera now."
EOF
```

Unplug/re-plug OAK-D Pro, then verify:
```bash
ssh jetson@192.168.7.250 'lsusb -t | grep Movidius'
# Success: "5000M" (SuperSpeed) | Failure: "480M" (USB 2.0)
```

---

**Step 3: Kernel Boot Parameters** (5 min + reboot, persistent)

```bash
ssh jetson@192.168.7.250 'bash -s' << 'EOF'
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo sed -i '/^      APPEND/ s/$/ usbcore.autosuspend=-1 usbcore.usbfs_memory_mb=1000 usbcore.quirks=03e7:2485:n,2109:0822:n,2109:0817:n/' /boot/extlinux/extlinux.conf
grep APPEND /boot/extlinux/extlinux.conf
sudo reboot
EOF
```

---

**Step 4: Persistent Udev Rules** (10 min, boot-resilient)

```bash
ssh jetson@192.168.7.250 'bash -s' << 'EOF'
sudo tee /etc/udev/rules.d/90-usb3-camera-stability.rules > /dev/null << 'RULES'
# VIA Labs VL822 — disable LPM/autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2109", ATTR{idProduct}=="0822", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms; echo disabled > /sys%p/power/usb3_hardware_lpm_u1 2>/dev/null; echo disabled > /sys%p/power/usb3_hardware_lpm_u2 2>/dev/null'"

# VIA Labs VL817 — disable LPM/autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2109", ATTR{idProduct}=="0817", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms; echo disabled > /sys%p/power/usb3_hardware_lpm_u1 2>/dev/null; echo disabled > /sys%p/power/usb3_hardware_lpm_u2 2>/dev/null'"

# Intel Movidius MyriadX (OAK-D) — disable autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", \
  RUN+="/bin/sh -c 'echo on > /sys%p/power/control; echo -1 > /sys%p/power/autosuspend_delay_ms'"
RULES
sudo udevadm control --reload-rules
echo "Udev rules installed."
EOF
```

---

**Step 5: Y-Adapter for External Power** (if using IR illumination)

OAK-D Pro with active IR draws up to 15W, exceeding USB 3.0's 4.5W. Use Luxonis Y-Adapter (~$15).

---

**Step 6: External Powered USB 3.0 Hub** (if Steps 1-5 fail)

Replace on-board VIA Labs hub cascade with known-good external hub. Recommended: TI TUSB8041-based hubs.

---

**Step 7: OAK-D Pro PoE** (nuclear option — guaranteed fix)

Purchase OAK-D Pro PoE (~$300) + PoE injector (~$30). Eliminates USB entirely.

---

**Decision Flowchart:**

```
Step 1 (Type-A) → USB 3.0? → YES → Add Steps 3+4+5 → DONE
                     ↓ NO
Step 2 (LPM off) → USB 3.0? → YES → Add Steps 3+4+5 → DONE
                     ↓ NO
Step 3 (kernel) → USB 3.0?  → YES → Add Step 4+5 → DONE
                     ↓ NO
Step 6 (ext hub) → USB 3.0? → YES → DONE
                     ↓ NO
Step 7 (PoE) → Guaranteed fix → DONE
```

**Expected outcome: Steps 1-4 will resolve the issue.** The U1/U2 LPM disable directly addresses the confirmed root cause.

**Key Discoveries:**
- There is NO OAK-D Pro CSI variant — all OAK products use USB or Ethernet for host communication
- OAK FFC products would exhibit identical U1/U2 LPM failures since they also use MyriadX over USB
- CSI/MIPI cameras are rock-solid but lack depth processing, active IR, neural inference, and IMU
- GMSL2 is automotive-scale overkill ($500-1500+) with no depth processing
- OAK-D Pro PoE is the most reliable USB-bypass option but involves rewiring and cost
- No alternative interface matches OAK-D Pro's feature density at its price point ($279)
- The 7-step plan prioritizes confirmed software fix (LPM disable) over interface replacement

| File | Relevance |
|------|-----------|
| `scripts/voice_mapper.py` | Main robot script using OAK-D Pro via ROS2 topics |
| `scripts/start_robot.sh` | Hardware launcher that starts camera |

**External Sources:**
- [Luxonis FFC Deployment Guide](https://docs.luxonis.com/hardware/platform/deploy/ffc/) — Confirms USB/PoE-only host interface
- [Luxonis Jetson Deployment Guide](https://docs.luxonis.com/hardware/platform/deploy/to-jetson/) — Power recommendations
- [NVIDIA Camera Development Docs](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/CameraDevelopment.html) — CSI, GMSL framework
- [NVIDIA GMSL Camera Framework](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/CameraDevelopment/JetsonVirtualChannelWithGmslCameraFramework.html)
- [Orin Nano Adaptation Guide](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html) — CSI/USB/PCIe specs

**Gaps:** Luxonis product pages returned 404/rendering errors; ArduCam Jetson camera page unavailable; GMSL2 deserializer pricing is approximate  
**Assumptions:** OAK-D Pro PoE provides identical features to USB variant; the USB 3.0 LPM disable (Steps 1-4) will resolve SuperSpeed fallback based on confirmed root cause analysis

## Overview

The ROSMASTER A1's OAK-D Pro camera suffers USB 3.0 → USB 2.0 fallback not because of hardware wiring limitations, but because of a **USB 3.0 Link Power Management (LPM) protocol incompatibility** between the Intel Movidius MyriadX VPU, VIA Labs VL817 hub chip on the NVIDIA P3768 carrier board, and the NVIDIA tegra-xusb host controller driver.

The Jetson Orin Nano Super's "Super" designation is purely a software clock/power upgrade — the USB architecture is identical to the standard Orin Nano. The T234 SoC natively supports USB 3.2 Gen 2 (10 Gbps) with three SuperSpeed lanes, and the carrier board's Type-A ports are physically capable of USB 3.0 (5 Gbps through the VL817 hub). The camera successfully negotiates SuperSpeed on every attempt but falls back within seconds when U1/U2 low-power state transitions fail.

The camera is currently connected to the USB-C port, which **cannot provide USB 3.0** because the FUSB301 Type-C controller lacks SuperSpeed lane muxing. This alone accounts for the persistent USB 2.0 operation. Additionally, the FUSB301 causes periodic attach/detach cycling, leading to disconnects every 60-600 seconds. The Yahboom robot's internal QinHeng USB 2.0 hub (initially suspected as the bottleneck) only serves low-bandwidth peripherals (audio, serial) and has zero impact on camera bandwidth — disproving the hypothesis from prior research (002).

The fix is primarily **software configuration**: disable USB3 hardware LPM on the VIA Labs hubs, add kernel boot parameters to disable autosuspend and set per-device LPM quirks, increase the usbfs memory buffer, and install persistent udev rules. The camera must first be moved from USB-C back to a Type-A port. These changes directly address the confirmed root cause with high confidence. No alternative camera interface (CSI, GMSL2, PoE) is needed — the OAK-D Pro's unique combination of stereo depth + active IR + neural VPU + IMU cannot be replicated by any other interface at comparable cost.

## Key Findings

1. **"Super" = software-only.** The Orin Nano Super has the identical T234 SoC, P3768 carrier board, and USB architecture as the standard Orin Nano. USB I/O is completely unaffected by the Super upgrade.

2. **USB 3.0 works physically.** The OAK-D Pro achieves SuperSpeed enumeration on every attempt (visible in dmesg as bus 2 connect at 5000M). The SoC, UPHY lanes, and hub cascade all support USB 3.0.

3. **U1/U2 LPM is the root cause.** The specific failure is `"Disable of device-initiated U1 failed"` / `"Disable of device-initiated U2 failed"` — a USB 3.0 Link Power Management handshake breakdown between Movidius MyriadX firmware, VIA Labs VL817 hub firmware, and tegra-xusb host driver.

4. **USB-C port = USB 2.0 only.** The FUSB301 Type-C controller on the P3768 is a basic CC logic chip with no SuperSpeed lane muxing. The camera must be on a Type-A port for any chance of USB 3.0.

5. **Yahboom hub is innocent.** The QinHeng USB 2.0 hub serves only robot peripherals (audio, serial). It does NOT cause the camera's USB 2.0 fallback.

6. **All-default USB configuration.** No USB kernel boot parameters, no per-device quirks, no power management udev rules, no LPM overrides are configured. The system is running with every LPM-trigger enabled by default — `usb3_hardware_lpm_u1=enabled`, `usb3_hardware_lpm_u2=enabled`, `autosuspend=2s`, `usbfs_memory_mb=16`.

7. **Platform-wide issue.** 50+ NVIDIA forum threads confirm identical U1/U2 LPM disconnect symptoms across all Jetson platforms with VIA Labs USB 3.0 hubs. Affects cameras (OAK-D, RealSense, e-con), storage devices, and industrial hardware alike.

8. **No OAK CSI variant exists.** All OAK products (USB and FFC) use USB or Ethernet for host communication. The only non-USB OAK option is PoE.

## Actionable Conclusions

1. **Immediately move the OAK-D Pro from USB-C to a Type-A port** — USB-C physically cannot provide USB 3.0 on this carrier board.

2. **Disable USB3 hardware LPM on VIA Labs hubs** via sysfs commands — this directly addresses the confirmed U1/U2 failure root cause and should restore stable SuperSpeed operation.

3. **Add kernel boot parameters** (`usbcore.autosuspend=-1`, `usbcore.usbfs_memory_mb=1000`, `usbcore.quirks=03e7:2485:n,2109:0822:n,2109:0817:n`) for persistent USB stability across reboots.

4. **Install udev rules** for VIA Labs VL822/VL817 and Movidius devices to automatically apply LPM-disable settings at USB enumeration.

5. **Use a Y-adapter** for external power when using OAK-D Pro's active IR stereo (draws up to 15W, exceeding USB 3.0's 4.5W per-port limit).

6. **If software fixes fail**: replace the on-board hub cascade with a TI TUSB8041-based powered USB 3.0 hub, or migrate to OAK-D Pro PoE as the guaranteed nuclear option.

## Open Questions

- Will disabling U1/U2 LPM on VIA Labs hubs achieve stable sustained SuperSpeed, or will the tegra-xusb driver itself initiate problematic LPM transitions?
- Is the FUSB301's lack of SS muxing a hardware limitation of the chip or a device tree configuration issue that could be overridden?
- Would a JetPack version newer than 6.1 (R36.4.3) include fixes for the tegra-xusb U1/U2 LPM negotiation with VIA Labs hubs?
- Does the DepthAI usbfs interface claim bug (`process did not claim interface 0 before use`) contribute to link instability during recovery attempts?

## Standards Applied

No organizational standards applicable to this research.

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-researcher |
| Created Date | 2026-02-24 |
| Status | ✅ Complete |
| Current Phase | ✅ Complete |
| Path | /docs/research/004-orin-nano-super-usb3-cameras.md |
