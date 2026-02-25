
# Autonomous Landing System: STM32 ↔ SpeedyBee Interface Plan  
**ZVALS – Group 10**

---

## Overview

The control team will use INAV firmware on the SpeedyBee F405 Mini flight controller.

INAV’s native MAVLink support allows the STM32F4 companion processor to send autonomous flight commands over a dedicated UART, while the RC transmitter remains fully connected and always retains manual override priority.

**No firmware modification is required.**

---

## System Architecture

### 1. Normal RC Flight (Unchanged)

- RC Transmitter → RF → RC Receiver → SBUS/CRSF → SpeedyBee UART (Serial RX)  
- Signal path is completely stock — identical to a standard FPV drone build  
- STM32 is fully idle during manual flight and not involved in this path  

---

### 2. Autoland Mode (STM32 Active)

- HackRF SDR → STM32F4 (FreeRTOS) — receives 3 beacon distance values  
- STM32 runs triangulation control algorithm  
- Computes: **Roll / Pitch / Throttle / Yaw**  
- STM32 → MAVLink over UART → SpeedyBee (separate UART from RC receiver)  
- INAV accepts `RC_CHANNELS_OVERRIDE` on flight axes  
- RC transmitter retains AUX/mode channels  

---

## Toggle Mechanism

- Pilot assigns a physical switch on RC transmitter to an AUX channel (e.g., AUX1)  
- **Switch OFF** → INAV uses RC input only; STM32 sends nothing  
- **Switch ON** → STM32 detects mode change via MAVLink telemetry feedback and begins sending autoland commands  
- Flipping switch back instantly restores full manual control (no software delay)  

---

## Hardware Connection

The STM32F4 connects to the SpeedyBee via 3 soldered wires on a free UART (UART2 or UART3).

| STM32F4 Pin | SpeedyBee Pad | Notes |
|-------------|--------------|-------|
| TX (UART3)  | RX2 or RX3  | STM32 transmits → FC receives |
| RX (UART3)  | TX2 or TX3  | FC transmits telemetry → STM32 receives |
| GND         | GND         | Common ground required |

- No USB connection needed between STM32 and SpeedyBee  
- 3.3V TTL logic — both chips are compatible; no level shifting required  
- Baud rate: 115200  

---

## INAV Configuration (One-Time Setup)

- **Ports tab** → Set chosen UART telemetry protocol to MAVLink, baud 115200  
- RC receiver remains on its own UART as normal Serial RX (unchanged)  
- **Modes tab** → Assign AUX channel to autoland trigger mode  
- No CLI modifications  
- No firmware recompilation  
- No hex file editing required  

---

## STM32 FreeRTOS Task Structure

- **Task 1: MAVLink RX**  
  - Reads telemetry from INAV (altitude, attitude, current flight mode)  

- **Task 2: RF Distance RX**  
  - Receives 3 beacon distances from HackRF  

- **Task 3: Triangulation / Control Algorithm**  
  - Computes autoland commands  

- **Task 4: MAVLink TX (50 Hz)**  
  - Only active when autoland mode detected  
  - Sends `RC_CHANNELS_OVERRIDE`  

**Key Detail:**  
Sending `UINT16_MAX` on any channel in `RC_CHANNELS_OVERRIDE` tells INAV to ignore that channel and defer to the RC transmitter. Only Roll, Pitch, Throttle, and Yaw are overridden during autoland.

---

## MAVLink Library for STM32

- Use official MAVLink C header-only library  
- Drop headers directly into STM32CubeIDE / PlatformIO project  
- No separate compilation step  
- No external dependencies  
- Fully compatible with FreeRTOS + HAL  

**STM32 Sends:**  
- `RC_CHANNELS_OVERRIDE` (Command ID #70)

**STM32 Receives:**  
- `HEARTBEAT`  
- `SYS_STATUS`  
- `ATTITUDE`  
- `ALTITUDE`  

---

## Why INAV Over Betaflight for This Use Case

- Native `RC_CHANNELS_OVERRIDE` support — no workaround required  
- RC receiver and MAVLink companion link coexist on separate UARTs without conflict  
- Manual override always takes hardware priority (meets Phase 2 spec requirement)  
- Bidirectional MAVLink — STM32 receives live telemetry for closed-loop control  
- Compatible with QGroundControl for monitoring during testing  

---

## Reference Links

- INAV MAVLink Telemetry Source: iNavFlight/inav – `mavlink.c`  
- INAV MAVLink Telemetry Docs: iNavFlight/inav – `Telemetry.md`  
- INAV RC via MAVLink (feature tracking): iNavFlight/inav – Issue #6571  
- INAV Latest Releases (MAVLink RC support confirmed): iNavFlight/inav – Releases  
- ELRS MAVLink Setup Guide: expresslrs.org/software/mavlink  
- SpeedyBee F405 Mini Stack – Product Page: speedybee.com  
- Controlling INAV drone via MAVLink + companion computer: iNavFlight/inav – Issue #10387  
