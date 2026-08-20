# PIXKIT Autonomous Shuttle — Control Framework v6
## Complete Deployment & Field-Testing Reference

> **Platform:** ROS 2 Humble · Ubuntu 22.04 · SocketCAN (`can4`) · 500 kbps
> **Package:** `pix_control_framework_v6_final.tar.gz`
> **Last updated:** 2026-06-10

---

## Table of Contents
1. [Architecture Overview](#1-architecture-overview)
2. [Bug Fix History (All Sessions)](#2-bug-fix-history-all-sessions)
3. [Step-by-Step Deployment](#3-step-by-step-deployment)
4. [Actuator Test Procedures](#4-actuator-test-procedures)
5. [CAN Signal Reference](#5-can-signal-reference)
6. [VCU State Machine](#6-vcu-state-machine)
7. [Algorithm Activation](#7-algorithm-activation)
8. [Monitoring & Diagnostics](#8-monitoring--diagnostics)
9. [Troubleshooting Guide](#9-troubleshooting-guide)
10. [Safety Procedures](#10-safety-procedures)

---

## 1. Architecture Overview

```
Algorithms (YOLO / Lane / Cruise)
        |  /pix/commands/<source>
Command Arbitrator  (priority queue, 50 Hz)
        |  /pix/raw_control_cmd
Safety Manager      (watchdog · e-stop · rate-limit)
        |  /pix/control_cmd
CAN TX Node         (DBC encode -> SocketCAN)
        |  CAN bus (500 kbps)
VCU Hardware        (Steering · Throttle · Brake · Gear · Park)
        ^  CAN bus
CAN RX Node         (SocketCAN -> DBC decode)
        |  /pix/vehicle_status
State Manager / Diagnostics / Logger
```

**Topic map:**

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/pix/commands/cruise_control` | actuator_test / user | Arbitrator |
| `/pix/commands/collision_avoidance` | YOLO node | Arbitrator |
| `/pix/raw_control_cmd` | Arbitrator | Safety Manager |
| `/pix/control_cmd` | Safety Manager | CAN TX |
| `/pix/vehicle_status` | CAN RX | State Mgr, Diagnostics |
| `/pix/system_state` | State Manager | All |
| `/diagnostics` | Diagnostics | (monitor) |

---

## 2. Bug Fix History (All Sessions)

### v6 — 2026-06-10 (THIS RELEASE) *** CRITICAL ***

#### BUG: Gear & Park Commands Silently Rejected by VCU

**Root cause — two separate bugs:**

**Bug A — `can_tx.py`: `Auto_Professional` was conditional**

```python
# BROKEN — AP=0 whenever idle; VCU drops to Standby
'Auto_Professional': 1 if any_en else 0,

# FIXED — AP=1 always; VCU stays in Autonomous Mode
'Auto_Professional': 1,   # Always 1
```

The VCU has two operating modes:

| Vehicle_ModeState | Value | Accepts |
|-------------------|-------|---------|
| Standby Mode | 3 | Steering OK  Brake OK  Gear NO  Park NO |
| **Auto Mode** | **1** | **All subsystems OK** |

Between publishing bursts `any_en` dropped to False -> `Auto_Professional=0` -> VCU reverted to
Standby -> gear/park silently ignored. Steering and brake still worked in Standby (which is why
those tests passed), masking this bug.

**Bug B — `actuator_test.py`: gear/park tests didn't activate Auto Mode**

The VCU transitions Standby -> Auto ONLY when `steer_en=1` is sent alongside `Auto_Professional=1`.
The old gear/park tests sent only `gear_en=True` — VCU never entered Auto Mode.

**Fix:** All gear/park test steps now include `steer_en=True, steer_target=0.0` plus a
**2-second VCU warm-up phase** at the start of each test.

**Wire-level proof:**

```
# Before fix — 0x105 byte[0]=0x00 = Auto_Professional=0
can4  105  [8]  00 01 00 00 00 00 00 01   <- AP=0, VCU in Standby

# After fix — 0x105 byte[0]=0x80 = Auto_Professional=1
can4  105  [8]  80 01 00 00 00 00 00 81   <- AP=1, VCU in Auto Mode

# Expected gear PARK command (0x103)
can4  103  [8]  01 01 00 00 00 00 00 00   <- Gear_EnCtrl=1, Gear_Target=1(PARK)

# Expected gear DRIVE command (0x103)
can4  103  [8]  01 04 00 00 00 00 00 05   <- Gear_EnCtrl=1, Gear_Target=4(DRIVE)

# Expected park ENGAGE (0x104)
can4  104  [8]  01 01 00 00 00 00 00 00   <- Park_EnCtrl=1, Park_Target=1(trigger)
```

---

### v5 — 2026-06-09

| Fix | File | Detail |
|-----|------|--------|
| CAN startup E-stop | `safety_manager_node.py` | 3 s grace period ignores VCU boot faults |
| CAN startup E-stop | `system_state_manager_node.py` | 3 s grace period |
| CAN TX buffer full | `can_tx.py` | errno 105 -> WARNING (not ERROR), log-throttled |
| Config file install | `pix_vehicle_interface/setup.py` | `glob('config/*.yaml')` in data_files |
| DBC decode crash | `dbc_decoder.py` | `decode_choices=False` + `_to_num()` fallback |

---

## 3. Step-by-Step Deployment

### 3.1 Transfer to Vehicle Computer

```bash
# USB stick
cp ~/Desktop/Manish/Custom_Interface_study/pix_control_framework_v6_final.tar.gz /media/usb/

# OR SCP over network
scp ~/Desktop/Manish/Custom_Interface_study/pix_control_framework_v6_final.tar.gz \
    sysadmin@<vehicle-ip>:~/Downloads/
```

### 3.2 Extract & Build (on Vehicle Computer)

```bash
cd ~/Downloads
tar -xzvf pix_control_framework_v6_final.tar.gz
cd pix_control_framework

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Expected: `Summary: 13 packages finished` — zero ERRORs.

### 3.3 Verify CAN Interface

```bash
ip link show can4

# If not up:
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000

# Confirm VCU frames
candump can4 | head -20
# Must see: 0x505 (VCU_Report), 0x502 (Steering_Report), 0x503 (Gear_Report)
```

### 3.4 Launch the Framework

```bash
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash
source install/setup.bash

# Full hardware mode
ros2 launch launch/hw_framework.launch.py profile:=hardware

# Steering-only safe test (no throttle/drive)
ros2 launch launch/hw_framework.launch.py profile:=hardware steer_only_mode:=True
```

### 3.5 Monitor Health (separate terminals)

```bash
# Terminal 2 — system state
ros2 topic echo /pix/system_state

# Terminal 3 — vehicle status
ros2 topic echo /pix/vehicle_status

# Terminal 4 — diagnostics
ros2 topic echo /diagnostics

# Terminal 5 — CAN dump
candump can4
```

**Expected healthy state after v6:**
```
vehicle_status:
  vehicle_mode_state: 1      <- Auto Mode (was 3=Standby in v5)
  auto_professional_fb: 1    <- VCU acknowledged AP mode
  steer_en_state: 1          <- Steer in Auto (was 3=Standby in v5)
  gear_actual: <changes>     <- Gear responds to commands now
```

---

## 4. Actuator Test Procedures

### 4.1 Run Tests

```bash
python3 scripts/actuator_test.py --mode hardware --test steering
python3 scripts/actuator_test.py --mode hardware --test brake
python3 scripts/actuator_test.py --mode hardware --test gear
python3 scripts/actuator_test.py --mode hardware --test park
python3 scripts/actuator_test.py --mode hardware --test throttle   # vehicle MOVES!
python3 scripts/actuator_test.py --mode hardware --test full
```

### 4.2 Gear Test — Expected CAN frames

Monitor in second terminal: `candump can4 | grep -E " 103 | 104 | 105 | 503 | 504 "`

| Test Step | 0x105 byte[0] | 0x103 payload | 0x503 Gear_Actual |
|-----------|---------------|---------------|-------------------|
| Warm-up | 0x80 (AP=1) | 01 01... | 1 (PARK) |
| NEUTRAL  | 0x80 (AP=1) | 01 03... | 3 (NEUTRAL) |
| DRIVE    | 0x80 (AP=1) | 01 04... | 4 (DRIVE) |
| NEUTRAL  | 0x80 (AP=1) | 01 03... | 3 (NEUTRAL) |
| PARK     | 0x80 (AP=1) | 01 01... | 1 (PARK) |

If 0x105 byte[0] = 0x00 -> AP=0 -> VCU in Standby -> rebuild with v6 fix.

### 4.3 Throttle Test Safety Checklist

- [ ] 5 m clear ahead
- [ ] Operator at E-stop button
- [ ] Gear in DRIVE (gear test must pass first)
- [ ] First run: max_speed = 1.0 m/s

---

## 5. CAN Signal Reference

### TX Messages (Laptop -> VCU)

#### 0x100 — Throttle_Command
| Signal | Bit | Note |
|--------|-----|------|
| Dirve_EnCtrl | 0, len=1 | Enable drive |
| Dirve_SpeedTarget | 32, len=16 | Speed target |
| Dirve_ThrottlePedalTarget | 16, len=16 | Pedal % |
| Dirve_Acc | 8, len=8 | m/s^2 |
| CheckSum_100 | 63, len=8 | XOR byte[0..6] |

#### 0x101 — Brake_Command
| Signal | Bit | Note |
|--------|-----|------|
| Brake_EnCtrl | 0, len=1 | Enable brake |
| Brake_Pedal_Target | 8, len=16 | 0-100% |
| Brake_Dec | 24, len=16 | m/s^2 |
| AEB_EnCtrl | 40, len=1 | AEB enable |
| CheckSum_101 | 63, len=8 | XOR byte[0..6] |

#### 0x102 — Steering_Command
| Signal | Bit | Note |
|--------|-----|------|
| Steer_EnCtrl | 0, len=1 | Enable steer |
| Steer_AngleTarget | 16, len=16 | deg signed |
| Steer_AngleSpeed | 32, len=16 | 1-250 deg/s |
| CheckSum_102 | 63, len=8 | XOR byte[0..6] |

#### 0x103 — Gear_Command
| Signal | Bit | Values |
|--------|-----|--------|
| Gear_EnCtrl | 0, len=1 | 0=Disable, 1=Enable |
| Gear_Target | 10, len=3 | 0=INVALID, 1=PARK, 2=REVERSE, 3=NEUTRAL, 4=DRIVE |
| CheckSum_103 | 63, len=8 | XOR byte[0..6] |

> IMPORTANT: Gear commands ONLY accepted when Vehicle_ModeState=1 (Auto Mode).
> Auto Mode requires Auto_Professional=1 in 0x105 AND Steer_EnCtrl=1 in 0x102.

#### 0x104 — Park_Command
| Signal | Bit | Values |
|--------|-----|--------|
| Park_EnCtrl | 0, len=1 | 0=Disable, 1=Enable |
| Park_Target | 8, len=1 | 0=Release, 1=Parking_trigger |
| CheckSum_104 | 63, len=8 | XOR byte[0..6] |

#### 0x105 — Vehicle_Mode_Command  *** CRITICAL ***
| Signal | Bit | Values |
|--------|-----|--------|
| Auto_Professional | 7, len=1 | **ALWAYS 1** (byte[0] = 0x80) |
| Steer_ModeCtrl | 2, len=3 | 0=Standard |
| Drive_ModeCtrl | 10, len=3 | 1=Speed Drive |
| TurnLight_Ctrl | 17, len=2 | 0=OFF,1=LEFT,2=RIGHT,3=HAZARD |
| Headlight_Ctrl | 18, len=1 | 0/1 |
| CheckSum_105 | 63, len=8 | XOR byte[0..6] |

### RX Messages (VCU -> Laptop)

#### 0x505 — VCU_Report (key fields)
| Signal | Healthy | Problem |
|--------|---------|---------|
| Vehicle_ModeState | 1 (Auto) | 3 = Standby |
| Auto_ProfessionalFb | 1 (Enable) | 0 = not in AP mode |
| CarPower_State | 2 (READY) | 0=OFF, 1=ON |
| CarWork_State | 4 (work) | 5=Estop, 6=error, 7=crash |

#### 0x502 — Steering_Report
| Signal | Healthy | Problem |
|--------|---------|---------|
| Steer_EnState | 1 (Auto) | 3 = Standby |
| Steer_Flt1/2 | 0 | 1 = fault |

#### 0x503 — Gear_Report
| Signal | Values |
|--------|--------|
| Gear_Actual | 0=INVALID, 1=PARK, 2=REVERSE, 3=NEUTRAL, 4=DRIVE |
| Gear_Flt | 0=No Fault |

#### 0x504 — Park_Report
| Signal | Values |
|--------|--------|
| Parking_Actual | 0=Release, 1=Parking_trigger |
| Park_Flt | 0=No Fault |

---

## 6. VCU State Machine

```
Power ON
  |
  v
Manual/Remote Mode (0)
  |  [Auto_Professional=1 + Steer_EnCtrl=1 received]
  v
Standby Mode (3) <----------------------------------+
  |  [Auto_Professional=1 continuous + steer_en=1]  |
  v                                                  |
Auto Mode (1)  <- gear/park commands accepted here  |
  |  [emergency or AP=0]                             |
  v                                                  |
Emergency Mode (2) ---------------------------------+
```

Key rule: Auto_Professional=1 must be CONTINUOUSLY held in 0x105.
If it ever drops to 0, VCU returns to Standby within ~100 ms.

---

## 7. Algorithm Activation

### Steering-Only Avoidance (Stationary)

```bash
ros2 launch launch/hw_framework.launch.py steer_only_mode:=True
```

### Full Avoidance with Motion

```bash
ros2 launch launch/hw_framework.launch.py steer_only_mode:=False
```

### Manual Commands via CLI

```bash
ros2 topic pub /pix/commands/cruise_control pix_vehicle_msgs/msg/PixControlCmd \
  '{steer_en: true, steer_target: 0.0, steer_speed: 100.0,
    gear_en: true, gear_target: 3,
    park_en: true, park_target: 0}'
```

---

## 8. Monitoring & Diagnostics

```bash
# Watch vehicle status
watch -n1 "ros2 topic echo /pix/vehicle_status --once"

# CAN — command frames only
candump can4 | grep -E " 100 | 101 | 102 | 103 | 104 | 105 "

# CAN — report frames only
candump can4 | grep -E " 500 | 501 | 502 | 503 | 504 | 505 "

# Check Auto_Professional byte live
candump can4 | awk '/ 105 / {if ($5=="80") print "AP=1 OK"; else print "AP=0 ERROR: " $0}'

# Log files
ls -lt ~/pix_logs/ | head -5
```

---

## 9. Troubleshooting Guide

### Gear stays at NEUTRAL (3) — doesn't change

1. `candump can4 | awk '/ 105 /'` — byte[0] must be `80`
2. `ros2 topic echo /pix/vehicle_status | grep vehicle_mode_state` — must be `1`
3. If VCU stays in Standby (3): verify `can_tx.py` has `'Auto_Professional': 1` (not conditional)
4. Verify gear test sends `steer_en=True`
5. Rebuild: `colcon build --symlink-install`

### Park brake doesn't engage

Same Auto Mode requirement as gear. Check 0x504 changes within 1-2 s of 0x104 command.

### E-stop triggers at launch

Boot-time transient. The 3 s grace period suppresses this. If persists:
```bash
grep startup_grace src/pix_safety_manager/pix_safety_manager/safety_manager_node.py
# Must show: self.startup_grace_period = 3.0
```

### CAN TX buffer warnings

Shown as WARNING in v6. Increase queue if frequent:
```bash
sudo ip link set can4 txqueuelen 2000
```

### PackageNotFoundError on launch

```bash
colcon build --symlink-install
source install/setup.bash
```

### Steering/Brake work but Gear/Park don't

Classic v5 symptom. Auto_Professional was conditional. Deploy v6 package.

---

## 10. Safety Procedures

### Pre-Drive Checklist

- [ ] All humans clear of vehicle (5 m radius)
- [ ] E-stop button tested manually
- [ ] `candump can4` shows VCU frames
- [ ] CarPower_State = 2 (READY)
- [ ] CarWork_State = 0 (init) or 4 (work)
- [ ] Gear in PARK before launch
- [ ] First throttle test: max 1.0 m/s in open space

### Software E-Stop

```bash
ros2 topic pub --once /pix/commands/emergency_stop \
  pix_vehicle_msgs/msg/PixControlCmd '{emergency_stop: true}'
```

Clear latch after E-stop:
```bash
pkill -f safety_manager_node
ros2 launch launch/hw_framework.launch.py profile:=hardware
```

---

## Appendix A — Modified Files (v6)

| File | Change |
|------|--------|
| `src/pix_vehicle_interface/pix_vehicle_interface/can_tx.py` | Auto_Professional=1 always (v6 critical fix) |
| `scripts/actuator_test.py` | Gear/park tests: steer_en=True + VCU warm-up (v6) |
| `src/pix_safety_manager/pix_safety_manager/safety_manager_node.py` | 3 s startup grace (v5) |
| `src/pix_state_manager/pix_state_manager/system_state_manager_node.py` | 3 s startup grace (v5) |
| `src/pix_vehicle_interface/pix_vehicle_interface/dbc_decoder.py` | decode_choices=False + _to_num() (v5) |
| `src/pix_vehicle_interface/setup.py` | glob('config/*.yaml') in data_files (v5) |
| `launch/hw_framework.launch.py` | steer_only_mode param (v5) |

## Appendix B — Version Log

| Version | Date | Key Change |
|---------|------|------------|
| v1-v3 | 2026-06-07 | Initial framework, basic CAN TX/RX |
| v4 | 2026-06-08 | DBC signal audit, NamedSignalValue fix |
| v5 | 2026-06-09 | Startup grace, buffer fix, setup.py config install |
| **v6** | **2026-06-10** | **Auto_Professional always=1; gear/park VCU Auto Mode fix** |
