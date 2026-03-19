# Adaptive Traffic Light Control System on FPGA

## Overview
This repository contains a full HDL hardware implementation of an **Adaptive Traffic Light Control System**, designed for the **Digilent Basys 3 (Xilinx Artix-7) FPGA**. 

Instead of relying strictly on fixed-time phases, the system leverages a Moore Finite State Machine to dynamically adjust the duration of green lights based on real-time traffic density data received from an external Camera AI module (via UART). It features robust handling for pedestrian crossings, anti-starvation mechanisms, and a hardware-level emergency vehicle override.

## Key Features
* **Adaptive Timing Logic:** The core data path calculates traffic density bonuses dynamically. It reads N-S and E-W vehicle density nibbles (0-15) to extend optimal green light durations (scaled between 8s and 45s).
* **Robust UART Camera Interface:** A custom asynchronous UART receiver (`115200 baud, 8N1`) actively listens for incoming density packet payloads `[0xAA, Data, 0x55]`. Includes a 5-second asynchronous watchdog that gracefully falls back to fixed timing (`15s` base) if the external AI camera disconnects or drops frames.
* **Pedestrian Priority System:** Push-button inputs with hardware debouncing log asynchronous pedestrian walk requests. When triggered, the system enforces a minimum green phase duration (`20s`) for the corresponding direction on the next available cycle to ensure safe crossing.
* **Crisp 7-Segment Multiplexing:** Features a custom binary-to-BCD decoded 7-segment multiplexer running at a ~763Hz sweep frequency. Displays live N-S countdown timers on the left two digits, and E-W countdowns on the right. Employs hardware blanking/dead-time to completely eliminate LED ghosting and bleed-over.
* **Emergency Override:** A master hardware switch instantly suspends normal operations, safely jumping the FSM into a hazard state that disables green lights and emits a clean, strictly timed 50% duty-cycle flashing yellow alert (0.5Hz) in all directions.
* **Starvation Prevention:** Hardcoded safety limits guarantee that regardless of extreme density biases or pedestrian spam, no traffic direction is ever starved of a green light for more than 50 seconds.

## Architecture & Modules
The hardware is strictly synchronous and hierarchical:
* `traffic_system_rx.v` — **Top-Level Wrapper**. Bootstraps the UART handler and feeds data into the Top module.
* `uart_camera_rx.v` — 16x oversampling UART receiver with a robust start/frame sync finite state machine.
* `traffic_system_top.v` — Instantiates the core timing and computational modules:
  * `traffic_controller_core.v` — The backbone FSM orchestrating light transitions, countdown timers, and emergency hazards.
  * `adaptive_control_core.v` — Pure combinational ALU tasked with calculating timing bonuses, evaluating priority parameters, and clamping to safe minimums/maximums.
  * `ped_request_handler.v` — State-aware debouncer securely holding pedestrian walk requests until they are fulfilled.
* `seg7_mux_driver.v` / `seg7_hex_decoder.v` — Multiplexed anode/cathode display drivers optimized for the Basys 3 hardware layout.

## Repository Structure
```text
├── rtl/                        # Synthesizable Hardware Code (Verilog)
├── tb/                         # Simulation Testbenches
├── sw/                         # PC testing utilities (C++)
│   └── uart_density_sender.cpp # Mock Camera AI for active serial line testing
├── xdc/                        # Hardware constraints
│   └── trafficlight.xdc        # Master hardware constraints (XDC) for Basys 3
├── Vivado/                     # Vivado Project Files (Ignored via .gitignore)
└── README.md
```

## I/O Hardware Mapping (Basys 3)
| I/O Port | Pin | Description |
|---|---|---|
| `clk` | W5 | 100MHz System Clock |
| `rst_n` | R2 | Master Reset (Active Low) — Uses SW15 |
| `emergency_sw` | V17 | Emergency Override Hazard Mode — Uses SW0 |
| `uart_rx` | B18 | RS232 Serial RX pin (receives live AI Camera data) |
| `ns_leds[2:0]` | U16, E19, U19 | North-South Traffic Lights (Red, Yellow, Green) |
| `ew_leds[2:0]` | V19, W18, U15 | East-West Traffic Lights (Red, Yellow, Green) |
| `ped_ns_led`  | U14 | North-South Pedestrian Walk Signal indicator |
| `ped_ew_led`  | V14 | East-West Pedestrian Walk Signal indicator |
| `seg_o[6:0]` | (Multiple) | 7-Segment Display Cathodes (Countdown visualizer) |
| `an_o[3:0]` | (Multiple) | 7-Segment Display Anodes (Digit scanners) |

*(Refer to `xdc/trafficlight.xdc` for full pin definitions).*

## PC UART Testing Utility
If you don't have a physical Camera AI module hooked up, a C++ application is provided to test the UART Adaptive pipeline from your computer.

**Compile via MinGW/GCC:**
```cmd
cd sw
g++ -o uart_density_sender.exe uart_density_sender.cpp
```
**Run:**
```cmd
uart_density_sender.exe COM3
```
* **Interactive Mode:** Simply type `<NS> <EW>` density (0-15) and hit Enter (e.g. `15 2`). Be aware that manual interactive packets expire after our 5-second `TIMEOUT_MS` threshold.
* **Auto Mode:** Type `auto <NS> <EW> <interval_ms>` to stream UART packets continuously. This perfectly simulates an active camera framerate and forces the FSM to ingest the new density data seamlessly gracefully. E.g., `auto 15 2 200`.

## Simulation
Run any of the provided testbenches using Icarus Verilog or Vivado Simulator (XSim). 
Example verifying the 7-segment display logic natively:
```bash
iverilog -o tb_seg7.vvp tb/tb_seg7_display.v rtl/seg7_hex_decoder.v rtl/seg7_mux_driver.v
vvp tb_seg7.vvp
```
