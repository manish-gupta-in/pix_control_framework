# PIX Control Framework v13 — Complete Usage & Reference Guide

## Table of Contents
1. [Architecture Overview](#1-architecture-overview)
2. [Unified Command Pipeline (v12 Fix)](#2-unified-command-pipeline-v12-fix)
3. [Hardware Limits & Safety Parameters](#3-hardware-limits--safety-parameters)
4. [Deploying on the Vehicle](#4-deploying-on-the-vehicle)
5. [Writing a New Algorithm](#5-writing-a-new-algorithm)
6. [How Messages Flow: Algorithm → VCU](#6-how-messages-flow-algorithm--vcu)
7. [Priority System & Adding New Priority Slots](#7-priority-system--adding-new-priority-slots)
8. [Actuator Testing (Steering, Braking, Throttle)](#8-actuator-testing-steering-braking-throttle)
9. [Diagnostic Tools](#9-diagnostic-tools)
10. [Unit Testing](#10-unit-testing)

---

## 1. Architecture Overview

The PIX Control Framework v13 is a modular, standard ROS2 workspace for autonomous control of the PIXKIT DTV shuttle. It provides:

- **C++ CAN Codec** (`pix_can_codec`): Hand-written, zero-allocation byte-level CAN encoder/decoder.
- **C++ CAN Driver** (`pix_can_driver`): Multi-threaded SocketCAN driver for CAN4.
- **C++ Vehicle Interface** (`pix_vehicle_interface_cpp`): 50Hz encode/decode loop — **pure encoder only, no arbitration**.
- **Python Command Arbitrator** (`pix_command_manager`): Priority-based command mux, config-driven.
- **Python Safety Manager** (`pix_safety_manager`): Rate-limiting, clamping, watchdog, E-stop.
- **Python Algorithm API** (`pix_algorithm_api`): Base class for all algorithms.
- **Custom Messages** (`pix_control_msgs`, `pix_vehicle_msgs`): standard-shaped reports and commands.

### Key Packages

| Package | Language | Role |
|---|---|---|
| `pix_can_codec` | C++ | CAN frame encode/decode |
| `pix_can_driver` | C++ | SocketCAN ↔ ROS2 bridge |
| `pix_vehicle_interface_cpp` | C++ | 50Hz CAN command loop (pure encoder) |
| `pix_command_manager` | Python | Priority arbitration + standard msg normalization |
| `pix_safety_manager` | Python | Safety clamping, watchdog, E-stop |
| `pix_algorithm_api` | Python | Base class for all algorithms |
| `pix_config_manager` | Python | Profile loader (hardware/simulation/tuning) |
| `pix_state_manager` | Python | System state machine |
| `pix_diagnostics` | Python | Health checks → /diagnostics |
| `pix_logger` | Python | CSV telemetry logging |

---

## 2. Unified Command Pipeline (v12 Fix)

### The Problem v12 Fixes
In v11, there were **two competing arbitration paths**:
1. Legacy algorithms → arbitrator → safety → interface
2. Standard algorithms → interface directly (bypassing safety & arbitrator!)

### The v12 Solution: Single Pipeline

```
┌─────────────────────────────────────────────────────┐
│ Algorithm A (Standard: pix_control_msgs/Control)    │
│ Algorithm B (Legacy: pix_vehicle_msgs/PixControlCmd)│
│ E-Stop (unconditional override)                     │
└────────────────────┬────────────────────────────────┘
                     ▼
          pix_command_manager (ARBITRATOR)
          ├── Accepts BOTH message types
          ├── Normalizes standard → PixControlCmd internally
          ├── Config-driven priority (YAML)
          ├── E-stop is unconditional override
          └── Preemption events logged
                     │
                     ▼  /pix/raw_control_cmd (exactly ONE topic)
          pix_safety_manager (CLAMP + WATCHDOG)
          ├── Steering: ±500° max, 250°/s rate limit
          ├── Speed: 0–5.0 m/s
          ├── Acceleration: 0–3.0 m/s²
          ├── Brake: 0–100%
          └── Watchdog: 300ms timeout → E-stop
                     │
                     ▼  /pix/control_cmd (exactly ONE topic)
          pix_vehicle_interface_cpp (PURE ENCODER)
          ├── NO arbitration
          ├── NO clamping
          ├── NO freshness comparison
          └── Just encodes PixControlCmd → CAN frames
                     │
                     ▼
          pix_can_driver → SocketCAN → VCU
```

**Rule:** Every algorithm command goes through **one arbitrator, one safety clamp, one interface**. No exceptions, no bypass.

---

## 3. Hardware Limits & Safety Parameters

These values are derived from the production DTV vehicle (hooke2_interface reference files):

| Parameter | Value | Source |
|---|---|---|
| Max Steering Angle | 500.0° (wheel) | hooke2 steering_command_102.cpp |
| Max Steering Rate | 250.0°/s | hooke2 steering_command_102.cpp |
| Max Speed | 5.0 m/s (~18 km/h) | hooke2 throttle_command_100.cpp |
| Max Acceleration | 3.0 m/s² | hooke2 brake_command_101.cpp |
| Max Brake | 100.0% | hooke2 brake_command_101.cpp |
| Steering Ratio | 16.6:1 (wheel:tire) | hooke2 calibration |
| CAN Bus | can4 @ 500 kbps | Vehicle hardware |
| Control Loop Rate | 50 Hz (20ms) | VCU requirement |

These are enforced in `pix_safety_manager` and configured in `launch/hw_framework.launch.py`:
```python
{
    'max_steer_angle':   500.0,
    'max_steer_rate':    250.0,
    'max_speed':           5.0,
    'max_accel':           3.0,
    'watchdog_timeout':    0.3,
}
```

---

## 4. Deploying on the Vehicle

### Step 1: Transfer to Vehicle
```bash
# On your dev machine
scp pix_control_framework_v13.zip vehicle_user@192.168.x.x:~/
```

### Step 2: Unzip and Build
```bash
# On the vehicle computer
cd ~
unzip pix_control_framework_v13.zip
cd pix_control_framework

# Build (can_msgs is pre-installed on the vehicle)
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 3: Bring Up CAN Interface
```bash
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000

# Verify VCU frames are coming in
candump can4 -n 5
# You should see frames on 0x500–0x512
```

### Step 4: Launch the Framework
```bash
# Terminal 1: Launch core framework
ros2 launch launch/hw_framework.launch.py

# Terminal 2: Run diagnostic watcher (verify all topics alive)
python3 scripts/pix_framework_watcher_node.py
```

### Step 5: VCU Pre-flight Checklist
Before gear/steering/throttle commands will work:
1. ⚠ **E-stop must be disengaged** (green LED on VCU panel)
2. ⚠ **Physical switch must be in AUTO** (not STANDBY)
3. Verify: `candump can4 | grep 505` shows byte[4] = `0x01` (Auto Mode)
4. Only THEN will the VCU accept ROS commands

---

## 5. Writing a New Algorithm

Every algorithm inherits from `BaseAlgorithmInterface`. You have **two ways** to publish commands:

### Method A: Standard Path (Recommended — standard)
Use **radians** for steering and **m/s** for speed. The arbitrator normalizes automatically.

```python
#!/usr/bin/env python3
import rclpy
import math
from pix_algorithm_api import BaseAlgorithmInterface

class MyAlgorithm(BaseAlgorithmInterface):
    def __init__(self):
        # The topic name determines your priority slot
        super().__init__('my_algorithm_node', '/pix/commands/lane_following')
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz

    def control_loop(self):
        status = self.get_vehicle_status()
        
        # Your perception/planning logic here...
        target_tire_angle_rad = 5.0 * math.pi / 180.0  # 5° tire angle
        target_speed_ms = 2.0  # 2 m/s
        target_accel = 1.0  # 1 m/s²
        
        # Publish using standard method (radians + m/s)
        self.publish_standard_control(
            steering_tire_angle=target_tire_angle_rad,
            steering_tire_rotation_rate=0.1,  # rad/s
            velocity=target_speed_ms,
            acceleration=target_accel,
            gear_command=3,  # DRIVE
        )

def main(args=None):
    rclpy.init(args=args)
    node = MyAlgorithm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What happens internally when you call `publish_standard_control()`:**
1. Your algorithm publishes `pix_control_msgs/Control` on `/pix_framework/control_cmd`
2. The **arbitrator** receives it and converts:
   - `steering_tire_angle` (rad) → `steer_target` (deg) via `angle_deg = (rad × 180/π) × steering_ratio`
   - `velocity` (m/s) → `speed_target` (m/s)
   - `acceleration` → positive = throttle, negative = brake
3. The normalized `PixControlCmd` competes in the priority queue
4. Winner goes to **safety manager** for clamping
5. Clamped command goes to **vehicle interface** for CAN encoding

### Method B: Legacy Path (Direct PixControlCmd)
Use **degrees** for steering directly. Published to your algorithm's registered topic.

```python
self.publish_control_cmd(
    steer_target=100.0,      # degrees (wheel angle)
    steer_speed=150.0,       # deg/s
    steer_en=True,
    speed_target=2.0,        # m/s
    accel_target=1.0,        # m/s²
    drive_en=True,
    brake_target=0.0,        # 0–100%
    brake_en=False,
    gear_target=3,           # DRIVE
    gear_en=True,
    park_target=0,           # RELEASE
    park_en=True,
)
```

**Both methods go through the same arbitrator → safety → interface pipeline. No bypass.**

---

## 6. How Messages Flow: Algorithm → VCU

### Complete Topic Flow Diagram

```
Algorithm Node
  │
  ├── publish_standard_control()
  │     → /pix_framework/control_cmd   (pix_control_msgs/Control)
  │     → /pix_framework/gear_cmd      (pix_control_msgs/GearCommand)
  │
  └── publish_control_cmd()
        → /pix/commands/<algorithm_name>  (pix_vehicle_msgs/PixControlCmd)
  
       ▼
pix_command_manager (arbitrator)
  │  Subscribes to:
  │    /pix_framework/control_cmd     ← standard path (normalized internally)
  │    /pix/commands/collision_avoidance  ← legacy path
  │    /pix/commands/human_avoidance     ← legacy path
  │    /pix/commands/lane_following      ← legacy path
  │    /pix/commands/cruise_control      ← legacy path
  │    /pix/commands/emergency_stop      ← unconditional override
  │
  │  Publishes:
  │    /pix/raw_control_cmd          (PixControlCmd — single winner)
  
       ▼
pix_safety_manager
  │  Subscribes to: /pix/raw_control_cmd
  │  Publishes to:  /pix/control_cmd     (clamped, rate-limited)
  
       ▼
pix_vehicle_interface_cpp
  │  Subscribes to: /pix/control_cmd     (ONLY input — single topic)
  │  Encodes → CAN frames
  │  Publishes: /pix_framework/can_rx    (to pix_can_driver)
  
       ▼
pix_can_driver → SocketCAN → VCU (physical vehicle)
```

### CAN Frame Mapping

| CAN ID | Name | Direction |
|--------|------|-----------|
| 0x100 | Throttle Command | PC → VCU |
| 0x101 | Brake Command | PC → VCU |
| 0x102 | Steering Command | PC → VCU |
| 0x103 | Gear Command | PC → VCU |
| 0x104 | Park Command | PC → VCU |
| 0x105 | VCU Mode Command | PC → VCU |
| 0x500 | Throttle Report | VCU → PC |
| 0x501 | Brake Report | VCU → PC |
| 0x502 | Steering Report | VCU → PC |
| 0x503 | Gear Report | VCU → PC |
| 0x504 | Park Report | VCU → PC |
| 0x505 | VCU Report | VCU → PC |
| 0x512 | BMS Report | VCU → PC |

---

## 7. Priority System & Adding New Priority Slots

### Current Priority Order (from `arbitrator_params.yaml`)

| Priority | Source Name | Topic | Message Type |
|----------|-----------|-------|------|
| **E-STOP** | EMERGENCY_STOP | `/pix/commands/emergency_stop` | Unconditional override — NOT in priority list |
| 1 (Highest) | STANDARD_CONTROL | `/pix_framework/control_cmd` | `pix_control_msgs/Control` |
| 2 | COLLISION_AVOIDANCE | `/pix/commands/collision_avoidance` | `PixControlCmd` |
| 3 | HUMAN_AVOIDANCE | `/pix/commands/human_avoidance` | `PixControlCmd` |
| 4 | LANE_FOLLOWING | `/pix/commands/lane_following` | `PixControlCmd` |
| 5 (Lowest) | CRUISE_CONTROL | `/pix/commands/cruise_control` | `PixControlCmd` |

### How to Add a New Algorithm Priority Slot

**Step 1:** Edit `src/pix_command_manager/config/arbitrator_params.yaml`:
```yaml
/**:
  ros__parameters:
    active_timeout: 0.4
    priority_sources:
      - "STANDARD_CONTROL"
      - "COLLISION_AVOIDANCE"
      - "HUMAN_AVOIDANCE"
      - "MY_NEW_ALGORITHM"          # ← Add your name here
      - "LANE_FOLLOWING"
      - "CRUISE_CONTROL"
    priority_topics:
      - "/pix_framework/control_cmd"
      - "/pix/commands/collision_avoidance"
      - "/pix/commands/human_avoidance"
      - "/pix/commands/my_new_algorithm"   # ← Add your topic here
      - "/pix/commands/lane_following"
      - "/pix/commands/cruise_control"
```

**Step 2:** In your algorithm's `__init__`, use the matching topic:
```python
super().__init__('my_new_algorithm_node', '/pix/commands/my_new_algorithm')
```

**Step 3:** Rebuild and relaunch. **No code changes needed** — just the YAML edit.

### How E-Stop Works
- E-stop is **NOT** part of the priority list — it's an **unconditional override**
- When `/pix/commands/emergency_stop` receives `emergency_stop=True`, it immediately:
  - Bypasses ALL priority scanning
  - Publishes `emergency_stop=True` to `/pix/raw_control_cmd`
  - Safety manager applies full brakes, zero speed, wheels center
- E-stop is **latched** — it stays active until manually cleared via `/pix/estop_clear`

### Preemption Logging
When a higher-priority algorithm starts overriding a lower-priority one, the arbitrator logs:
```
[WARN] PREEMPTION EVENT: [LANE_FOLLOWING] overridden by [COLLISION_AVOIDANCE]
```
This appears in the terminal running `hw_framework.launch.py`.

---

## 8. Actuator Testing (Steering, Braking, Throttle)

With `hw_framework.launch.py` running in one terminal, open another:

### Test Steering Only
```bash
python3 scripts/actuator_test.py --mode hw --test steering
```
Ramps the steering column left and right. Safe — vehicle doesn't move.

### Test Braking Only
```bash
python3 scripts/actuator_test.py --mode hw --test brake
```
Tests hydraulic brake pressure. Safe — vehicle doesn't move.

### Test Throttle (⚠ VEHICLE WILL MOVE)
```bash
python3 scripts/actuator_test.py --mode hw --test throttle
```
⚠ **Ensure 15m of clear flat space ahead!**

### Full Test Suite (⚠ VEHICLE WILL MOVE)
```bash
python3 scripts/actuator_test.py --mode hw --test full
```
Tests Gear → Park → Throttle → Steering → Braking in sequence.

### Simulation Mode (No CAN, safe for bench)
```bash
python3 scripts/actuator_test.py --mode sim --test full
```

---

## 9. Diagnostic Tools

### Topic Watcher
```bash
python3 scripts/pix_framework_watcher_node.py
```
Checks:
- ✓ All standard report topics exist (`/pix_framework/vehicle_status/*`)
- ✓ CAN I/O topics exist (`/pix_framework/can_tx`, `can_rx`)
- ✓ Legacy pipeline topics exist (`/pix/control_cmd`, `/pix/raw_control_cmd`)
- ✓ **ZERO collisions** with external stack topics (`/control/command/*` etc.)

### Live CAN Debugging
```bash
# Watch raw CAN traffic
candump can4

# Watch only steering reports
candump can4,502:7FF

# Watch only throttle commands
candump can4,100:7FF
```

### ROS2 Topic Monitoring
```bash
# Check if arbitrator is publishing
ros2 topic hz /pix/raw_control_cmd

# Check if safety manager is publishing
ros2 topic hz /pix/control_cmd

# Check vehicle feedback
ros2 topic echo /pix_framework/vehicle_status/velocity
ros2 topic echo /pix_framework/vehicle_status/steering
```

---

## 10. Unit Testing

### Run All Tests (on dev machine or vehicle)
```bash
cd pix_control_framework
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run all Python tests
python3 -m pytest src/pix_command_manager/test/test_arbitration_logic.py \
                  src/pix_safety_manager/test/test_safety_and_arbitration.py -v
```

### What the Tests Cover

**`test_arbitration_logic.py` (4 tests):**
- Standard message (`pix_control_msgs/Control`) is correctly normalized to `PixControlCmd`
- E-stop unconditionally overrides all active sources simultaneously
- Priority order loads from YAML config correctly
- Preemption events are logged when higher priority overrides lower

**`test_safety_and_arbitration.py` (27 tests):**
- Steering clamp: ±350° range, boundary values
- Speed clamp: 0–5.0 m/s
- Acceleration clamp: 0–2.0 m/s²
- Brake clamp: 0–100%
- Steering rate limiting at 150°/s
- Full command validation pipeline
- E-stop command generation
- Arbitration priority ordering (pure-logic tests)

### Expected Output
```
31 passed in 0.28s
```
