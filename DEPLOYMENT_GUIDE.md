# PIXKIT Control Framework — Complete Deployment Guide
*Version 8 · Framework for PIX Moving Hooke2 Shuttle*

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Architecture](#2-architecture)
3. [First-Time Setup on Vehicle PC](#3-first-time-setup-on-vehicle-pc)
4. [CAN Bus Setup (Required Every Boot)](#4-can-bus-setup-required-every-boot)
5. [Build & Install](#5-build--install)
6. [Physical VCU Prerequisites](#6-physical-vcu-prerequisites)
7. [Launch the Core Interface](#7-launch-the-core-interface)
8. [Commissioning — Actuator Tests (in order)](#8-commissioning--actuator-tests-in-order)
9. [Expected Results for Each Test](#9-expected-results-for-each-test)
10. [Launching Algorithms](#10-launching-algorithms)
11. [Troubleshooting](#11-troubleshooting)
12. [Signal Reference](#12-signal-reference)
13. [Adding a New Algorithm](#13-adding-a-new-algorithm)

---

## 1. System Overview

```
Physical World ←──────────── CAN Bus (can4, 500 kbps) ──────────────→ Shuttle VCU
                              0x100 Throttle TX   0x500 Throttle RX
                              0x101 Brake TX      0x501 Brake RX
                              0x102 Steer TX      0x502 Steer RX
                              0x103 Gear TX       0x503 Gear RX
                              0x104 Park TX       0x504 Park RX
                              0x105 Mode TX       0x505 VCU Status RX
                                                  0x512 BMS (Battery) RX

ROS Topics:
  /pix/vehicle_status     ← decoded CAN data (50 Hz)
  /pix/control_cmd        → encoded to CAN TX (50 Hz)
  /pix/raw_control_cmd    ← algorithm commands (before safety filter)
  /pix/commands/<algo>    ← each algorithm's command topic
  /pix/system_state       → STANDBY / AUTONOMOUS / FAULT
  /pix/estop_trigger      → trigger E-stop
  /pix/estop_clear        → clear E-stop latch
```

---

## 2. Architecture

```
  ALGORITHM LAYER (launch separately, hot-swap)
  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
  │  YOLO Avoidance  │  │  Lane Following  │  │  Your Algorithm  │
  │ /pix/commands/   │  │ /pix/commands/   │  │ /pix/commands/   │
  │  human_avoidance │  │  lane_following  │  │  cruise_control  │
  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
           └──────────────────────┼──────────────────────┘
                                  ▼
  CORE INTERFACE LAYER (hw_framework.launch.py — always running)
  ┌─────────────────────────────────────────────────────────────┐
  │  Command Arbitrator (priority MUX: estop>collision>human>   │
  │                      cruise>lane)                           │
  │       ↓ /pix/raw_control_cmd                                │
  │  Safety Manager (rate-limit, bounds check, watchdog)        │
  │       ↓ /pix/control_cmd                                    │
  │  CAN TX Node (encode → 6 CAN frames @ 50 Hz)               │
  │       ↕ SocketCAN (can4)                                    │
  │  CAN RX Node (decode ← VCU frames → /pix/vehicle_status)   │
  │  System State Manager (/pix/system_state)                   │
  │  Diagnostics (/diagnostics)                                 │
  │  Logger (~/pix_logs/)                                       │
  │  Config Manager (/pix/config/active_profile)                │
  └─────────────────────────────────────────────────────────────┘
```

---

## 3. First-Time Setup on Vehicle PC

### 3.1 Install ROS 2 Humble (if not installed)
```bash
# Ubuntu 22.04 only
sudo apt update && sudo apt install ros-humble-ros-base python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3.2 Install Python Dependencies
```bash
pip3 install python-can cantools ultralytics torch torchvision
```

### 3.3 Transfer the Package
```bash
# From development PC
scp pix_control_framework_v8_final.tar.gz sysadmin@<vehicle-ip>:~/Downloads/

# On vehicle PC
cd ~/Downloads
tar -xzvf pix_control_framework_v8_final.tar.gz
# Results in: ~/Downloads/pix_control_framework/
```

---

## 4. CAN Bus Setup (Required Every Boot)

> ⚠️ This must be done after **every reboot** before launching the framework.

```bash
# Bring up the CAN interface at 500 kbps
sudo ip link set can4 up type can bitrate 500000

# Set TX queue length (prevents errno 105 / ENOBUFS drops)
sudo ip link set can4 txqueuelen 1000

# Verify it is up
ip link show can4
# Expected: <NOARP,UP,LOWER_UP,ECHO>  mtu 16  ...
```

### Verify VCU frames are arriving
```bash
candump can4 -n 20
# Must see frames with IDs: 0x500, 0x501, 0x502, 0x503, 0x504, 0x505, 0x512
# If no frames: check CAN cable, VCU power, bitrate (500k)
```

### Make CAN persistent across reboots (optional)
```bash
# Create systemd service
sudo nano /etc/systemd/network/80-can.network
# Content:
# [Match]
# Name=can4
# [CAN]
# BitRate=500000

sudo systemctl enable systemd-networkd
sudo systemctl restart systemd-networkd
```

---

## 5. Build & Install

```bash
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash

# Build all 13 packages
colcon build --symlink-install

# Expected output:
# Summary: 13 packages finished [~30s]
# (0 errors, some UserWarning about tests_require is normal)

# Source the workspace
source install/setup.bash
```

---

## 6. Physical VCU Prerequisites

> ⚠️ **CRITICAL FOR GEAR CONTROL** — Read before testing gear/throttle.

The VCU has a **hardware mode selector** (physical key or remote switch):

| VCU Switch Position | `Vehicle_ModeState` | Gear Commands | Brake/Steer |
|---------------------|---------------------|---------------|-------------|
| **STANDBY** | 3 | ❌ Blocked | ✅ Work |
| **AUTO** | 1 | ✅ Work | ✅ Work |
| **MANUAL/REMOTE** | 0 | ❌ Blocked | ❌ Blocked |
| **EMERGENCY** | 2 | ❌ Blocked | ❌ Blocked |

**Steps before testing gear or throttle:**
1. Disengage e-stop button (green LED on VCU panel)
2. Set the physical mode selector to **AUTO** position
3. Verify in ROS: `ros2 topic echo /pix/vehicle_status | grep vehicle_mode`
   - Must see `vehicle_mode: 1` ← AUTO, gear commands work
   - If `vehicle_mode: 3` ← STANDBY, gear is blocked by hardware

**Verify via candump:**
```bash
# Watch 0x505 VCU_Report. Check byte[4] bits[3:2]:
# 00 = MANUAL, 01 = AUTO, 10 = EMERGENCY, 11 = STANDBY
candump can4 | grep " 505 "
# Look for: ... 505 [8] XX XX XX XX X1 XX XX XX  (byte[4] = ?1 = AUTO)
# vs            ... 505 [8] XX XX XX XX X3 XX XX XX  (byte[4] = ?3 = STANDBY)
```

---

## 7. Launch the Core Interface

**Terminal 1 — Core CAN Interface (keep running):**
```bash
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch core (NO algorithms — they start separately)
ros2 launch launch/hw_framework.launch.py profile:=hardware
```

**Expected startup output:**
```
[can_rx-1]  CAN RX Node Initialized.
[can_tx-2]  Opened SocketCAN interface: can4
[command_arbitrator-3]  Arbitration State Change: [NONE] -> [STANDBY/NONE]
[safety_manager-4]  Safety Manager Node Initialized.
[system_state_manager-5]  State: MANUAL → STANDBY  (DBW engaged by VCU)
```

**Verify system is healthy (Terminal 2):**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic echo /diagnostics
```
Expected:
- `CAN RX — vehicle_status`: OK (< 50 ms)
- `CAN TX — control_cmd`: OK (< 5 ms)
- `VCU Faults`: No VCU faults
- `Battery (BMS)`: OK: V=~83V SOC=~87%

---

## 8. Commissioning — Actuator Tests (in order)

> Run these **one at a time**, in the order shown, from a separate terminal.
> Always confirm the previous test passes before running the next.

```bash
# In a new terminal:
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash && source install/setup.bash
```

### Test 1: Steering ✅ (known working)
```bash
python3 scripts/actuator_test.py --mode hw --test steering
```
**What happens:** Wheels move right (200°), left (−200°), return to center.
**What to check:** Wheels physically turn. `steer=±200.0°` in VCU feedback.

---

### Test 2: Brake ✅ (known working)
```bash
python3 scripts/actuator_test.py --mode hw --test brake
```
**What happens:** Brake pedal applies to 30%, releases to 0%.
**What to check:** Feel/hear brake engage. `brake=30.0%` in VCU feedback.

---

### Test 3: Gear 🔲 (requires VCU in AUTO mode)
```bash
# PREREQUISITE: Set VCU physical switch to AUTO mode first!
python3 scripts/actuator_test.py --mode hw --test gear
```
**What happens:**
1. 5s warm-up with brake+steer → gear shifts to NEUTRAL
2. Cycles: PARK(1) → NEUTRAL(3) → DRIVE(4) → NEUTRAL(3) → PARK(1)
3. Each step held for 4 seconds

**What to check:**
```
VCU → gear=3 | mode=AUTO✓(1)   ← correct
VCU → gear=4 | mode=STANDBY⚠(3) ← hardware interlock active, switch to AUTO
```
**Expected in VCU status:** `gear_actual` changes from 4 to 3 to 1 etc.

---

### Test 4: Park Brake 🔲
```bash
python3 scripts/actuator_test.py --mode hw --test park
```
**What happens:**
1. 5s warm-up → shift to NEUTRAL with brake
2. Shift to PARK gear (1) with brake
3. Release parking brake (`park_target=0`) → `park_actual=0`
4. Engage parking brake (`park_target=1`) → `park_actual=1`

**What to check:** `park_actual` changes 0→1 in VCU feedback.
> Note: Park brake engage/release was already confirmed working.

---

### Test 5: Throttle 🔲 (VEHICLE MOVES — clear 10m path!)
```bash
# PREREQUISITE: VCU in AUTO mode, 10m+ clear area, gear in NEUTRAL/PARK
python3 scripts/actuator_test.py --mode hw --test throttle
```
**What happens:**
1. 5s warm-up: brake + steer + gear=DRIVE (4)
2. Release brake, ramp to 1.5 m/s for 3s
3. Apply brake 30%, coast to stop in DRIVE
4. Shift NEUTRAL → PARK gear, engage park brake

**What to check:** Vehicle moves forward at ≈1.5 m/s.
> ⚠️ Speed fix in v8: Previously sending 0.375 m/s due to /4.0 bug. Now sends 1.5 m/s correctly.

---

### Test 6: Full Sequence (optional, after all individual tests pass)
```bash
python3 scripts/actuator_test.py --mode hw --test full
```
Runs all tests in sequence with interactive prompts between each.

---

## 9. Expected Results for Each Test

| Test | VCU Signal | Expected Value | Pass |
|------|------------|----------------|------|
| Steering RIGHT | `steer_angle` | −200.0° (negative=right for this VCU) | ✅ Verified |
| Steering LEFT | `steer_angle` | +191.0° (left) | ✅ Verified |
| Brake apply | `brake_pedal` | 28–30% | ✅ Verified |
| Brake release | `brake_pedal` | 0% | ✅ Verified |
| Gear NEUTRAL | `gear_actual` | 3 | 🔲 Needs AUTO mode |
| Gear DRIVE | `gear_actual` | 4 | 🔲 Needs AUTO mode |
| Gear PARK | `gear_actual` | 1 | 🔲 Needs AUTO mode |
| Park engage | `park_actual` | 1 | ✅ Verified |
| Park release | `park_actual` | 0 | ✅ Verified |
| Throttle 1.5 m/s | `vehicle_speed` | ~1.5 m/s | 🔲 Not yet tested |

---

## 10. Launching Algorithms

After the core interface is running, start algorithms in separate terminals.
**The core keeps running even if an algorithm crashes.**

### YOLO Person Avoidance (steer-only, safe for stationary)
```bash
# Terminal 3
ros2 launch launch/algorithms/yolo_avoidance.launch.py
```

### YOLO with full drive enabled
Edit `launch/algorithms/yolo_avoidance.launch.py`:
```python
'steer_only_mode': False,   # allow vehicle to move
'target_speed': 1.5,        # m/s
```
Then relaunch.

### Lane Following
```bash
ros2 launch launch/algorithms/lane_following.launch.py
```

### Custom Algorithm (for future integration)
Any ROS 2 node that publishes `PixControlCmd` to `/pix/commands/cruise_control` will work:
```python
from pix_vehicle_msgs.msg import PixControlCmd
publisher = node.create_publisher(PixControlCmd, '/pix/commands/cruise_control', 10)
cmd = PixControlCmd()
cmd.steer_en = True
cmd.steer_target = 30.0    # degrees
cmd.drive_en = True
cmd.speed_target = 1.0     # m/s
cmd.gear_en = True
cmd.gear_target = 4         # DRIVE
publisher.publish(cmd)
```

---

## 11. Troubleshooting

### ❌ Gear not changing (gear_actual stays at 4)
1. **Check VCU mode:** `vehicle_mode` must be 1 (AUTO)
   - If 3 (STANDBY): Set physical VCU switch to AUTO position
2. **Check brake:** All gear commands must include `brake_en=True`
3. **Check speed:** Vehicle must be stationary (`vehicle_speed = 0.0`)
4. **Verify CAN frame:** `candump can4 | grep " 103 "` — must see `01 0X...` not `00 00...`

### ❌ Gear stuck, `vehicle_mode: 3` always
- Physical VCU mode switch not in AUTO position
- This is a hardware interlock — software cannot override it
- Solution: Physically toggle the mode selector to AUTO

### ❌ CAN TX buffer full (errno 105)
```bash
sudo ip link set can4 txqueuelen 2000
```

### ❌ `package 'pix_safety_manager' not found`
```bash
source install/setup.bash
# then re-run the launch command
```

### ❌ E-stop latched, vehicle won't move
```bash
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool '{data: true}' --once
```

### ❌ Watchdog timeout errors in can_tx
- Algorithm is publishing too slowly (< 3.3 Hz)
- Check algorithm health: `ros2 topic hz /pix/commands/<algo>`
- Restart algorithm node

### ❌ steer_angle reads −499° (wrong)
- `Steer_AngleActual` has offset=−500 in DBC → actual 0° = raw 500 = decoded 0 ✅
- Value of −499° means raw=1 → actual = 1−500 = −499 which is approximately 0° (center)
- This is a DBC offset issue in the report frame — **physically the wheel is at center**

### ❌ Vehicle speed = 0 despite throttle command
- Check gear is in DRIVE (4) — speed command ignored unless in DRIVE
- Check VCU mode is AUTO (1)
- Speed was previously wrong due to /4.0 bug (fixed in v8) — rebuild required

---

## 12. Signal Reference

### CAN TX Commands (Our messages → VCU)

| CAN ID | Message | Key Signals |
|--------|---------|-------------|
| 0x100 | `Throttle_Command` | `Dirve_EnCtrl` (0/1), `Dirve_SpeedTarget` (0–40.95 m/s), `Dirve_Acc` (m/s²) |
| 0x101 | `Brake_Command` | `Brake_EnCtrl` (0/1), `Brake_Pedal_Target` (0–100%) |
| 0x102 | `Steering_Command` | `Steer_EnCtrl` (0/1), `Steer_AngleTarget` (−500 to +500°), `Steer_AngleSpeed` (0–250 °/s) |
| 0x103 | `Gear_Command` | `Gear_EnCtrl` (0/1), `Gear_Target` (0=INVALID, 1=PARK, 2=REVERSE, 3=NEUTRAL, 4=DRIVE) |
| 0x104 | `Park_Command` | `Park_EnCtrl` (0/1), `Park_Target` (0=RELEASE, 1=ENGAGE) |
| 0x105 | `Vehicle_Mode_Command` | `Auto_Professional` (always=1), `Drive_ModeCtrl` (1=SPEED), `Steer_ModeCtrl` (0=STANDARD) |

### CAN RX Reports (VCU → Our messages)

| CAN ID | Message | Key Signals |
|--------|---------|-------------|
| 0x500 | `Throttle_Report` | `Dirve_EnState` (0=OFF,1=AUTO,3=STANDBY), `Dirve_ThrottlePedalActual` |
| 0x501 | `Brake_Report` | `Brake_EnState`, `Brake_PedalActual` (0–100%) |
| 0x502 | `Steering_Report` | `Steer_EnState`, `Steer_AngleActual` (°, offset=−500) |
| 0x503 | `Gear_Report` | `Gear_Actual` (1=PARK, 2=REV, 3=NEUTRAL, 4=DRIVE) |
| 0x504 | `Park_Report` | `Parking_Actual` (0=released, 1=engaged) |
| 0x505 | `VCU_Report` | `Vehicle_ModeState` (1=AUTO, 3=STANDBY), `Vehicle_Speed` (m/s), `Auto_ProfessionalFb` |
| 0x512 | `BMS_Report` | `Battery_Voltage` (V), `Battery_Current` (A), `Battery_Soc` (%) |

### Gear Target Values
```
0 = INVALID  (do not use)
1 = PARK
2 = REVERSE
3 = NEUTRAL
4 = DRIVE
```

### VCU Mode States
```
0 = MANUAL/REMOTE (joystick)
1 = AUTO ← required for gear changes
2 = EMERGENCY
3 = STANDBY ← default, gear blocked
```

### En State Values (for Steer/Drive/Brake reports)
```
0 = OFF (DBW disabled)
1 = AUTO (DBW active, closed-loop)
2 = FAULT
3 = STANDBY (DBW on but not in auto control)
```

---

## 13. Adding a New Algorithm

### Step 1: Create your algorithm package
```bash
cd src/algorithms/
ros2 pkg create my_avoidance_algo --build-type ament_python --dependencies rclpy pix_vehicle_msgs
```

### Step 2: Subscribe to vehicle status, publish commands
```python
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus
import rclpy
from rclpy.node import Node

class MyAlgorithm(Node):
    def __init__(self):
        super().__init__('my_algorithm')
        # Subscribe to vehicle feedback
        self.status_sub = self.create_subscription(
            PixVehicleStatus, '/pix/vehicle_status', self.status_cb, 10)
        # Publish to interface — use cruise_control topic (lowest priority)
        self.cmd_pub = self.create_publisher(
            PixControlCmd, '/pix/commands/cruise_control', 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz minimum

    def status_cb(self, msg):
        self.speed = msg.vehicle_speed
        self.steer = msg.steer_angle
        self.mode = msg.vehicle_mode   # Must be 1 for gear/throttle

    def control_loop(self):
        cmd = PixControlCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.steer_en = True
        cmd.steer_target = 0.0     # Your steering logic here
        cmd.drive_en = True
        cmd.speed_target = 1.0     # m/s
        cmd.gear_en = True
        cmd.gear_target = 4        # DRIVE
        self.cmd_pub.publish(cmd)
```

### Step 3: Create launch file
Copy `launch/algorithms/yolo_avoidance.launch.py` and modify.

### Step 4: Register in arbitrator (if new topic needed)
Edit `src/pix_command_manager/config/arbitrator_params.yaml` to add your topic.

### Step 5: Run
```bash
# Terminal 1: Core interface (already running)
ros2 launch launch/hw_framework.launch.py

# Terminal 2: Your algorithm
ros2 run my_avoidance_algo my_algo_node
```

---

## Quick Reference Card

```
# EVERY SESSION:
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000

# LAUNCH CORE:
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch launch/hw_framework.launch.py profile:=hardware

# TESTS (in a new terminal after sourcing):
python3 scripts/actuator_test.py --mode hw --test steering
python3 scripts/actuator_test.py --mode hw --test brake
python3 scripts/actuator_test.py --mode hw --test gear      # Needs VCU AUTO mode
python3 scripts/actuator_test.py --mode hw --test park
python3 scripts/actuator_test.py --mode hw --test throttle  # Vehicle moves!

# MONITOR:
ros2 topic echo /pix/vehicle_status
ros2 topic echo /diagnostics
ros2 topic echo /pix/system_state
candump can4 | grep " 505 "   # Check VCU mode

# CLEAR E-STOP:
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool '{data: true}' --once

# LAUNCH ALGORITHM:
ros2 launch launch/algorithms/yolo_avoidance.launch.py
```
