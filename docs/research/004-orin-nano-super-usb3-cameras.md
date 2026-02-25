---
id: "004"
type: research
title: "NVIDIA Jetson Orin Nano Super — USB 3.0 Camera Attachment Deep Dive"
status: 🔄 In Progress
created: "2026-02-24"
current_phase: "1 of 5"
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
| 2 | Carrier Board USB Topology & Port Identification | ⏳ Not Started | NVIDIA Developer Kit carrier board schematic/design guide; which physical ports are USB 3.x vs 2.0; USB-C port capabilities (host vs device vs dual-role); internal USB hub ICs used on reference carrier; how Yahboom ROSMASTER A1 carrier may differ; community teardowns and port mapping | — |
| 3 | USB 3.0 Camera Compatibility & Attachment Methods | ⏳ Not Started | Web research on attaching USB 3.0 cameras to Jetson Orin Nano Super; OAK-D Pro (Luxonis) USB 3 on Jetson — known issues and solutions; Intel RealSense on Jetson USB 3; industrial USB3 Vision cameras; cable requirements (USB-C to USB-C, USB-A 3.0); power delivery considerations; bandwidth sharing when multiple USB 3 devices are connected | — |
| 4 | Community Solutions & Known Issues | ⏳ Not Started | NVIDIA Developer Forums posts about USB 3 cameras on Orin Nano; Luxonis community/docs for OAK-D on Jetson; common failure modes (USB 2.0 fallback, bandwidth throttling, power brownout); kernel/device-tree USB configuration; udev rules; JetPack version-specific USB fixes; USB 3.0 hub recommendations for Jetson | — |
| 5 | Alternative Interfaces & Best Practices Summary | ⏳ Not Started | CSI/MIPI camera interface vs USB 3.0 — pros/cons on Orin Nano Super; OAK-D Pro FFC (fixed-focus camera) CSI variant availability; GMSL2 camera support; practical recommendation matrix for camera attachment on this platform; summary of actionable steps for the ROSMASTER A1 robot | — |

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

**Status:** ⏳ Not Started  
**Session:** —

## Phase 3: USB 3.0 Camera Compatibility & Attachment Methods

**Status:** ⏳ Not Started  
**Session:** —

## Phase 4: Community Solutions & Known Issues

**Status:** ⏳ Not Started  
**Session:** —

## Phase 5: Alternative Interfaces & Best Practices Summary

**Status:** ⏳ Not Started  
**Session:** —

## Overview

*To be written after all phases complete.*

## Key Findings

*To be populated after all phases complete.*

## Actionable Conclusions

*To be populated after all phases complete.*

## Open Questions

*To be populated after all phases complete.*

## Standards Applied

No organizational standards applicable to this research.

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-researcher |
| Created Date | 2026-02-24 |
| Status | 🔄 In Progress |
| Current Phase | 0 of 5 |
| Path | /docs/research/004-orin-nano-super-usb3-cameras.md |
