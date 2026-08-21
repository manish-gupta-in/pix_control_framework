# PIXKIT Control Framework — Complete Deployment Guide
*Version 9 · PIX Moving Hooke2 Shuttle · Last updated: 2026-06-22*

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Architecture](#2-architecture)
3. [Speed & Safety Limits](#3-speed--safety-limits)
4. [First-Time Setup on Vehicle PC](#4-first-time-setup-on-vehicle-pc)
5. [CAN Bus Setup (Required Every Boot)](#5-can-bus-setup-required-every-boot)
6. [Build & Install](#6-build--install)
7. [Physical VCU Prerequisites](#7-physical-vcu-prerequisites)
8. [Launch the Core Interface](#8-launch-the-core-interface)
9. [Commissioning Tests — In Order](#9-commissioning-tests--in-order)
   - [Test 1: Steering](#test-1-steering--)
   - [Test 2: Brake](#test-2-brake--)
   - [Test 3: Gear](#test-3-gear--)
   - [Test 4: Park](#test-4-park--)
   - [Test 5: Throttle](#test-5-throttle--)
10. [Launching Algorithms](#10-launching-algorithms)
11. [Troubleshooting](#11-troubleshooting)
12. [Signal Reference](#12-signal-reference)
13. [Adding a New Algorithm](#13-adding-a-new-algorithm)

---

## 1. System Overview

```
Physical World ←──────── CAN Bus (can4, 500 kbps) ────────→ Shuttle VCU (Hooke2)
                          0x100 Throttle TX      0x500 Throttle RX
                          0x101 Brake TX         0x501 Brake RX
                          0x102 Steer TX         0x502 Steer RX
                          0x103 Gear TX          0x503 Gear RX
                          0x104 Park TX          0x504 Park RX
                          0x105 Mode TX          0x505 VCU Status RX
                                                 0x512 BMS (Battery) RX

ROS Topics:
  /pix/vehicle_status     ← decoded VCU feedback (50 Hz)
  /pix/control_cmd        → encoded → CAN TX frames (50 Hz)
  /pix/commands/<algo>    ← algorithm command inputs
  /pix/system_state       → STANDBY / AUTONOMOUS / FAULT
  /pix/estop_trigger      → trigger E-stop
  /pix/estop_clear        → clear E-stop latch
```

---

## 2. Architecture

```
  ALGORITHM LAYER (launch separately — hot-swappable)
  ┌───────────────────┐  ┌──────────────────┐  ┌──────────────────┐
  │  YOLO Avoidance   │  │  Lane Following  │  │  Your Algorithm  │
  │ /pix/commands/    │  │ /pix/commands/   │  │ /pix/commands/   │
  │  human_avoidance  │  │  lane_following  │  │  cruise_control  │
  └─────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
            └─────────────────────┼──────────────────────┘
                                  ▼
  CORE INTERFACE LAYER (hw_framework.launch.py — always running)
  ┌──────────────────────────────────────────────────────────────┐
  │  Command Arbitrator   (priority MUX)                         │
  │  Safety Manager       (rate-limit, bounds, watchdog 300 ms)  │
  │  CAN TX Node          (6 raw frames @ 50 Hz)                 │
  │  CAN RX Node          (decode VCU → /pix/vehicle_status)     │
  │  System State Manager (/pix/system_state)                    │
  │  Diagnostics / Logger                                        │
  └──────────────────────────────────────────────────────────────┘
```

---

## 3. Speed & Safety Limits

> All limits are enforced in TWO places: `can_tx.py` (frame level) and `safety_manager` (ROS level).

| Parameter | Value | Where set |
|-----------|-------|-----------|
| **Max speed (safety_manager)** | **3.0 m/s** (≈ 10.8 km/h) | `launch/hw_framework.launch.py` `max_speed` |
| **Max speed (can_tx software cap)** | **15.0 m/s** | `can_tx.py` build_throttle() |
| **Max speed (DBC hardware limit)** | **40.95 m/s** | `Dirve_SpeedTarget` 12-bit field |
| **Test target speed** | **1.5 m/s** (≈ 5.4 km/h) | `actuator_test.py` `THROTTLE_TARGET` |
| **Max steer angle** | **280.0°** | `safety_manager` `max_steer_angle` |
| **Max steer rate** | **150.0 °/s** | `safety_manager` `max_steer_rate` |
| **Test steer angle** | **200.0°** | `actuator_test.py` `STEER_RIGHT_TARGET` |
| **Test brake level** | **30.0%** | `actuator_test.py` `BRAKE_LEVEL` |
| **Gear shift brake** | **100.0%** | `actuator_test.py` (gear/throttle tests) |
| **Watchdog timeout** | **300 ms** | `safety_manager` — if no cmd → brake |
| **CAN TX rate** | **50 Hz** | `can_tx.py` |
| **Max brake decel** | **10.0 m/s²** | `Brake_Dec` DBC field |

### To change test speed (edit `actuator_test.py` line ~47):
```python
THROTTLE_TARGET = 1.5   # m/s — change this for your test
THROTTLE_ACCEL  = 0.5   # m/s² — how fast to reach target speed
HOLD_SECONDS    = 3.0   # seconds to hold each command
```

### To change safety limit (edit `launch/hw_framework.launch.py` line ~150):
```python
'max_speed': 3.0,          # m/s — hard limit in safety manager
'max_steer_angle': 280.0,  # degrees
```

---

## 4. First-Time Setup on Vehicle PC

### 4.1 Install ROS 2 Humble (Ubuntu 22.04)
```bash
sudo apt update && sudo apt install ros-humble-ros-base python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 4.2 Install Python Dependencies
```bash
pip3 install python-can cantools ultralytics torch torchvision
```

### 4.3 Transfer the Package
```bash
# From dev PC
scp pix_control_framework_v9_final.tar.gz sysadmin@<vehicle-ip>:~/Downloads/

# On vehicle PC
cd ~/Downloads
tar -xzvf pix_control_framework_v9_final.tar.gz
# → creates ~/Downloads/pix_control_framework/
```

---

## 5. CAN Bus Setup (Required Every Boot)

> ⚠️ Run this after **every reboot**, before launching the framework.

```bash
# Bring up CAN at 500 kbps
sudo ip link set can4 up type can bitrate 500000

# Prevent TX buffer overflow (errno 105)
sudo ip link set can4 txqueuelen 1000

# Verify it is UP
ip link show can4
# Expected: <NOARP,UP,LOWER_UP,ECHO>
```

### Verify VCU is sending frames
```bash
candump can4 -n 20
# Must see: 0x500, 0x501, 0x502, 0x503, 0x504, 0x505, 0x512
# If nothing: check CAN cable, VCU power, bitrate (must be 500k)
```

### Verify 0x103 Gear checksum is correct (v9 fix)
```bash
# During gear test, 0x103 for NEUTRAL must be: 01 03 00 00 00 00 00 04
# Byte[7] = 0x04 = SUM checksum (NOT XOR=0x02 which was the v8 bug)
candump can4 | grep " 103 "
```

---

## 6. Build & Install

```bash
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash

colcon build --symlink-install
# Expected: Summary: 13 packages finished

source install/setup.bash
```

---

## 7. Physical VCU Prerequisites

> ⚠️ **CRITICAL — read before any gear/throttle test.**

| VCU Switch | `Vehicle_ModeState` in 0x505 | Gear/Throttle |
|------------|------------------------------|---------------|
| **STANDBY** | 3 | ❌ Blocked |
| **AUTO** ← required | 1 | ✅ Works |
| MANUAL | 0 | ❌ Blocked |
| EMERGENCY | 2 | ❌ Blocked |

**Before gear/throttle tests:**
1. Disengage E-stop (green LED on VCU panel)
2. Set physical mode selector → **AUTO**
3. Verify: `ros2 topic echo /pix/vehicle_status | grep vehicle_mode`
   - `vehicle_mode: 1` = AUTO ✅
   - `vehicle_mode: 3` = STANDBY ← fix switch

---

## 8. Launch the Core Interface

**Terminal 1 — keep this running throughout all tests:**
```bash
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch launch/hw_framework.launch.py profile:=hardware
```

**Expected output:**
```
[can_rx-1]   CAN RX Node Initialized.
[can_tx-2]   Opened SocketCAN interface: can4
[can_tx-2]   CAN TX Node Initialized.
[system_state_manager-5]  State: MANUAL → STANDBY
```

**Check system health (Terminal 2):**
```bash
ros2 topic echo /diagnostics
# Expect: CAN RX OK, CAN TX OK, No VCU faults
```

---

## 9. Commissioning Tests — In Order

> Open a new terminal for each test. Source setup first:
> ```bash
> source /opt/ros/humble/setup.bash && source install/setup.bash
> ```
> Run tests **one at a time, in order**. Each must pass before the next.

---

### Test 1: Steering ✅

```bash
python3 scripts/actuator_test.py --mode hw --test steering
```

**Sequence:** Center → RIGHT 200° → LEFT -200° → Center  
**Hold:** 3 s per step  
**Pass criteria:**
- Wheels physically turn right and left
- `steer_angle: ±200` in VCU feedback

---

### Test 2: Brake ✅

```bash
python3 scripts/actuator_test.py --mode hw --test brake
```

**Sequence:** 0% → 30% brake → hold → release to 0%  
**Pass criteria:**
- Feel/hear brake engage
- `brake_pedal: 28–30%` in VCU feedback

---

### Test 3: Gear ✅ (fixed in v9)

> ⚠️ VCU physical switch must be in **AUTO** mode.

```bash
python3 scripts/actuator_test.py --mode hw --test gear
```

**Sequence (mirrors `gear.py` proven method):**

| Sub-step | Duration | Command | Check |
|----------|----------|---------|-------|
| Step 0 | 5 s | 100% brake + steer + throttle wake | `brake_en=Auto, drive_en=Auto, steer_en=Auto` |
| NEUTRAL | 4 s | gear_target=3 | `gear_actual=3` |
| DRIVE | 4 s | gear_target=4 | `gear_actual=4` |
| NEUTRAL | 4 s | gear_target=3 | `gear_actual=3` |
| REVERSE | 4 s | gear_target=2 | `gear_actual=2` |
| NEUTRAL | 4 s | gear_target=3 | `gear_actual=3` |

> **Note:** PARK gear (id=1) via 0x103 is NOT supported on this VCU.
> Park brake is controlled by 0x104 (see Test 4).

**v9 fix — why gear now works:**
- Old bug: `0x103` used XOR checksum → sent byte[7]=0x02 for NEUTRAL → VCU silently rejected
- Fix: `0x103` now uses SUM checksum → byte[7]=0x04 for NEUTRAL → VCU accepts ✅

**Verify with candump during test:**
```bash
candump can4 | grep " 103 "
# NEUTRAL: 01 03 00 00 00 00 00 04   ← SUM cs=0x04 ✓
# DRIVE:   01 04 00 00 00 00 00 05   ← SUM cs=0x05 ✓
# REVERSE: 01 02 00 00 00 00 00 03   ← SUM cs=0x03 ✓
```

---

### Test 4: Park ✅

```bash
python3 scripts/actuator_test.py --mode hw --test park
```

**Sequence:**
1. 5 s warm-up: 100% brake + NEUTRAL + park RELEASED
2. Park ENGAGE (`park_target=1` → 0x104 byte1=0x01)
3. Park RELEASE (`park_target=0` → 0x104 byte1=0x00)

**Pass criteria:**
- `park_actual: 1` (engaged), `park_actual: 0` (released)
- Physically: hear/feel park actuator click

---

### Test 5: Throttle 🔲 (vehicle moves!)

> ⚠️ **VEHICLE MOVES. 15 m+ clear flat path required. Wheel chocks in until Step 2.**

**Default speed: 1.5 m/s (≈ 5.4 km/h)**  
Safety manager hard cap: 3.0 m/s

```bash
python3 scripts/actuator_test.py --mode hw --test throttle
```

**Full sequence (mirrors `gear.py` mode_throttle):**

| Step | Duration | What happens | Check |
|------|----------|-------------|-------|
| Step 0/6 | 5 s | 100% brake + all subsystem wake | `drive_en_state=1 (Auto)` |
| Step 1/6 | 3 s | DRIVE gear (brake still 100%) | `gear_actual=4` ← **must confirm** |
| Step 2/6 | 1 s | Brake → 0% ← **remove chocks** | vehicle may begin to roll |
| Step 3/6 | 3 s | Speed ramps to 1.5 m/s | `vehicle_speed ≈ 1.5 m/s` |
| Step 4/6 | 3 s | Hold 1.5 m/s | `vehicle_speed` stays ~1.5 |
| Step 5/6 | 3 s | Speed=0, 100% brake to stop | `vehicle_speed = 0.00` |
| Step 6/6 | 4 s | NEUTRAL + park engage | `gear=3, park=1` |

**Verify CAN frames during Step 3 (speed=1.5 m/s):**
```bash
candump can4 | grep " 100 "
# 01 19 00 00 00 09 60 83
# └── EnCtrl=1   └────┘ SpeedTarget=1.5 m/s encoded (raw=150=0x096)
```

**If vehicle does not move at Step 3:**
```bash
ros2 topic echo /pix/vehicle_status
# drive_en_state: 1  → Auto (VCU accepting speed command) ✅
# drive_en_state: 3  → Standby (VCU not ready — check gear_actual=4)
# gear_actual: 4     → DRIVE ✅
# gear_actual: 3     → still NEUTRAL — gear didn't shift (check VCU mode=1)
```

**Safety: Ctrl+C anytime** → framework automatically sends 100% brake + NEUTRAL.

---

## 10. Launching Algorithms

After core interface is running, start algorithms separately:

```bash
# YOLO person avoidance
ros2 launch launch/algorithms/yolo_avoidance.launch.py

# Lane following
ros2 launch launch/algorithms/lane_following.launch.py
```

The core interface keeps running even if an algorithm crashes.

---

## 11. Troubleshooting

### ❌ Gear not changing (stays at 4)
1. VCU mode switch not in AUTO → set to **AUTO**
2. Wrong checksum → ensure you're on **v9** (check `can_tx.py` uses `_sum7` for 0x103)
3. brake_en not Auto → run Step 0 warm-up first (100% brake for 5 s)
4. Verify: `candump can4 | grep " 103 "` → byte[7] must be `04` for NEUTRAL, not `02`

### ❌ Vehicle doesn't move (throttle test)
1. `gear_actual` must be `4` (DRIVE) before releasing brake
2. `drive_en_state` must be `1` (Auto)
3. `vehicle_mode` must be `1` (AUTO)
4. Speed encoding: v9 sends SpeedTarget directly in m/s via Speed Drive mode (Drive_ModeCtrl=1)

### ❌ CAN TX buffer full (errno 105)
```bash
sudo ip link set can4 txqueuelen 2000
```

### ❌ E-stop latched
```bash
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool '{data: true}' --once
```

### ❌ Watchdog timeout (can_tx warns every 2 s)
- Algorithm publishing slower than 3.3 Hz
- Check: `ros2 topic hz /pix/commands/cruise_control`

---

## 12. Signal Reference

### CAN TX (Our PC → VCU)

| ID | Message | Key signals | Checksum |
|----|---------|-------------|----------|
| 0x100 | Throttle_Command | `Dirve_EnCtrl` (0/1), `Dirve_SpeedTarget` (0–40.95 m/s, scale=0.01), `Dirve_Acc` (m/s²) | **SUM** |
| 0x101 | Brake_Command | `Brake_EnCtrl` (0/1), `Brake_Pedal_Target` (0–100%, scale=0.1) | **XOR** |
| 0x102 | Steering_Command | `Steer_EnCtrl` (0/1), `Steer_AngleTarget` (−500 to +500°, raw=angle+500), `Steer_AngleSpeed` (0–250 °/s) | **XOR** |
| 0x103 | Gear_Command | `Gear_EnCtrl` (0/1), `Gear_Target` (2=REV, 3=NEU, 4=DRV) | **SUM** ← critical |
| 0x104 | Park_Command | `Park_EnCtrl` (0/1), `Park_Target` (0=RELEASE, 1=ENGAGE) | **SUM** |
| 0x105 | Vehicle_Mode_Command | `Auto_Professional`=1, `Drive_ModeCtrl`=1 (Speed Drive) | **SUM** |

> **Checksum rule:** byte[7] = algorithm applied to bytes[0:6]
> - SUM: `sum(bytes[0:6]) & 0xFF`
> - XOR: `bytes[0] ^ bytes[1] ^ ... ^ bytes[6]`

### Gear Target Values

```
0 = INVALID   (never send)
1 = PARK      (NOT supported via 0x103 on this VCU — use 0x104 instead)
2 = REVERSE
3 = NEUTRAL
4 = DRIVE
```

### CAN RX (VCU → Our PC)

| ID | Message | Key signals |
|----|---------|-------------|
| 0x500 | Throttle_Report | `Dirve_EnState` (0=Manual,1=Auto,3=Standby) |
| 0x501 | Brake_Report | `Brake_EnState`, `Brake_PedalActual` (%) |
| 0x502 | Steering_Report | `Steer_EnState`, `Steer_AngleActual` (°, offset=−500) |
| 0x503 | Gear_Report | `Gear_Actual` (2=REV,3=NEU,4=DRV) |
| 0x504 | Park_Report | `Parking_Actual` (0=released,1=engaged) |
| 0x505 | VCU_Report | `Vehicle_ModeState` (1=AUTO,3=STANDBY), `Vehicle_Speed` (m/s) |
| 0x512 | BMS_Report | `Battery_Voltage`, `Battery_Soc` (%) |

### En State Values (0x500/0x501/0x502)
```
0 = Manual   (DBW disabled)
1 = Auto     ← required for autonomous control
2 = Takeover
3 = Standby  (DBW on but not in auto)
```

---

## 13. Adding a New Algorithm

### Step 1: Publish to a command topic
```python
from pix_vehicle_msgs.msg import PixControlCmd

cmd = PixControlCmd()
cmd.header.stamp = node.get_clock().now().to_msg()
cmd.steer_en     = True
cmd.steer_target = 0.0      # degrees
cmd.drive_en     = True
cmd.speed_target = 1.0      # m/s (safety_manager caps at 3.0 m/s)
cmd.gear_en      = True
cmd.gear_target  = 4        # DRIVE
cmd.brake_en     = False
cmd.brake_target = 0.0
pub.publish(cmd)
```
Publish to `/pix/commands/cruise_control` at **≥ 10 Hz** (watchdog = 300 ms).

### Step 2: Run alongside core
```bash
# Terminal 1: core (keep running)
ros2 launch launch/hw_framework.launch.py profile:=hardware

# Terminal 2: your algorithm
ros2 run my_package my_node
```

---

## Quick Reference Card

```bash
# ── EVERY SESSION ──────────────────────────────────────────
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000

# ── LAUNCH CORE ────────────────────────────────────────────
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch launch/hw_framework.launch.py profile:=hardware

# ── TESTS (new terminal, after sourcing) ───────────────────
python3 scripts/actuator_test.py --mode hw --test steering   # ✅ no movement
python3 scripts/actuator_test.py --mode hw --test brake      # ✅ no movement
python3 scripts/actuator_test.py --mode hw --test gear       # ✅ no movement (fixed v9)
python3 scripts/actuator_test.py --mode hw --test park       # ✅ no movement
python3 scripts/actuator_test.py --mode hw --test throttle   # ⚠️  VEHICLE MOVES @ 1.5 m/s

# ── MONITOR ────────────────────────────────────────────────
ros2 topic echo /pix/vehicle_status          # full feedback
candump can4 | grep " 103 "                  # verify gear frames
candump can4 | grep " 505 "                  # VCU mode state

# ── CLEAR E-STOP ───────────────────────────────────────────
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool '{data: true}' --once

# ── ALGORITHM ──────────────────────────────────────────────
ros2 launch launch/algorithms/yolo_avoidance.launch.py
```

---

## Version History

| Version | Key Change |
|---------|-----------|
| v9 | **Gear checksum fix** — 0x103 now uses SUM (not XOR). Gear now works. Throttle test mirrors gear.py. |
| v8 | Speed encoding fixed (removed /4.0 bug), modular algorithm launch |
| v7 | VCU hardware interlock documented, actuator_test diagnostic output |
