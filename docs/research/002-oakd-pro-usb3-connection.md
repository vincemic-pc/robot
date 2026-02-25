---
id: "002"
type: research
title: "OAK-D Pro USB 3.0 Connection on ROSMASTER A1 / Jetson Orin Nano"
status: 🔄 In Progress
created: "2026-02-24"
current_phase: "0 of 5"
---

## Introduction

The OAK-D Pro camera (Intel Movidius MyriadX, USB ID 03e7:2485) is currently connected via USB 2.0 at 480Mbps on the ROSMASTER A1 robot, yielding ~10fps instead of the full 30fps achievable over USB 3.0. The robot's `lsusb -t` output shows two USB buses: **Bus 02** (USB 3.0, 10000M) with a hub chain but no devices, and **Bus 01** (USB 2.0, 480M) where all devices including the camera are connected. All physical USB ports tried so far route through Bus 01. This research will investigate the USB topology, Jetson Orin Nano carrier board layout, OAK-D Pro cable requirements, and practical solutions to achieve a USB 3.0 connection.

## Objectives

- Determine the Yahboom ROSMASTER A1 USB port layout and internal hub topology
- Identify which physical port(s) on the Jetson Orin Nano carrier board provide USB 3.0 (Bus 02)
- Understand OAK-D Pro USB 3.0 cable and power requirements
- Determine whether the Jetson Orin Nano USB-C port maps to Bus 02 and supports peripherals
- Identify practical solutions: cable routing, hub bypass, USB-C connection, or firmware workarounds
- Provide actionable recommendation for getting OAK-D Pro running at USB 3.0 speeds

## Research Phases

| Phase | Name | Status | Scope | Session |
|-------|------|--------|-------|---------|
| 1 | Jetson Orin Nano USB 3.0 Port Identification | ⏳ Not Started | Research Jetson Orin Nano Developer Kit carrier board USB layout; identify USB-C vs USB-A 3.0 ports; determine which port maps to Bus 02 (10000M); check USB-C peripheral mode vs host mode; review NVIDIA documentation and pinout diagrams | — |
| 2 | ROSMASTER A1 USB Hub Topology & Port Routing | ⏳ Not Started | Research Yahboom ROSMASTER A1 internal USB wiring; identify if an internal USB 2.0 hub routes all external ports; determine hub chip model; check Yahboom schematics or community teardowns; determine if any external port connects directly to Jetson USB 3.0 bus | — |
| 3 | OAK-D Pro USB 3.0 Cable & Power Requirements | ⏳ Not Started | Research Luxonis OAK-D Pro USB 3.0 cable requirements (USB-C to USB-C vs USB-C to USB-A); minimum cable quality/length; power delivery requirements over USB 3.0; known compatibility issues with Jetson platforms; Luxonis documentation and community reports | — |
| 4 | Live Robot USB Topology Deep Dive | ⏳ Not Started | SSH into robot and run detailed USB diagnostics: full `lsusb -t` tree, `dmesg` USB enumeration logs, `/sys/bus/usb/devices/` inspection, identify hub vendor/product IDs, check if USB 3.0 bus has any connected hub/port physically accessible, test USB-C port with peripheral | — |
| 5 | Practical Solutions & Recommendations | ⏳ Not Started | Synthesize findings from Phases 1-4; evaluate options: (a) USB-C to USB-C direct connection to Jetson, (b) bypass internal hub with USB 3.0 extension cable, (c) external USB 3.0 hub on Jetson native port, (d) software/firmware workarounds for USB 2.0 throughput; assess feasibility, cost, and risk of each; provide ranked recommendation | — |

## Phase 1: Jetson Orin Nano USB 3.0 Port Identification

**Status:** ⏳ Not Started  
**Session:** —

## Phase 2: ROSMASTER A1 USB Hub Topology & Port Routing

**Status:** ⏳ Not Started  
**Session:** —

## Phase 3: OAK-D Pro USB 3.0 Cable & Power Requirements

**Status:** ⏳ Not Started  
**Session:** —

## Phase 4: Live Robot USB Topology Deep Dive

**Status:** ⏳ Not Started  
**Session:** —

## Phase 5: Practical Solutions & Recommendations

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

No organizational standards applicable to this research (hardware connectivity investigation).

## Handoff

| Field | Value |
|-------|-------|
| Created By | pch-researcher |
| Created Date | 2026-02-24 |
| Status | 🔄 In Progress |
| Current Phase | 0 of 5 |
| Path | /docs/research/002-oakd-pro-usb3-connection.md |
