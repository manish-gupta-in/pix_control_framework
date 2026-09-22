# PIX Control Framework v19
### Hooke2 PIXKIT DTV Shuttle — Production Autonomous Control Stack

> **Framework version:** v19 | **ROS2:** Humble | **Vehicle:** PIXKIT DTV Hooke2  
> **Single command format:** `PixControlCmd` (pix_vehicle_msgs) — degrees, m/s, percent

---

## Quick Start (2 terminals)

```bash
# ─── Terminal 1: Core hardware pipeline ───────────────────────────────────
source /opt/ros/humble/setup.bash
cd ~/pix_control_framework
colcon build --symlink-install && source install/setup.bash
ros2 launch launch/hw_framework.launch.py

# ─── Terminal 2: Autonomy stack ────────────────────────────────────────────
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch pix_autonomy autonomy_test.launch.py
```

> ⚠ **Before launching:** VCU physical switch must be in **AUTO** and e-stop must be **disengaged**.  
> ⚠ **Only ever run ONE arbitrator.** It starts automatically in Terminal 1.

---

## Documents

| File | Purpose |
|---|---|
| `README.md` (this file) | Quick start, architecture, test procedures |
| `PIX_FRAMEWORK_V15_USAGE.md` | Full API reference, field units, priority system, how to write an algorithm |
| `PIX_AEB_STRAIGHT_TEST_MANUAL.md` | Step-by-step field test manual (straight drive + AEB) |
| `pix_autonomy_architecture.md` | Architecture diagram, complete algorithm example node |
| `scripts/actuator_test.py` | Individual actuator tests (steering / brake / throttle / gear) |

## Version History

| Version | Date | Description |
|---|---|---|
| **v19.0** | 2026-09 | Major refactor: `aeb_node` enhanced (min_trigger_speed, min_distance params); `straight_drive_node` WAKE state machine (safe VCU startup); `base_algorithm_interface` expanded API; `control_arbitrator_node.py` removed (arbitration in `pix_command_manager`); `Control.msg`/`GearCommand.msg` removed from `pix_control_msgs`; `command_arbitrator`, `lane_following`, `yolo_avoidance` updated. |
| v18.0 | 2026-09 | `pix_autonomy/setup.py`: added missing `control_arbitrator_node` entry point; `test_imports.py` import coverage extended. |

---

## System Architecture

```
Algorithm nodes (pix_autonomy)
  └── publish_control_cmd()
        → /pix/commands/<name>   (PixControlCmd — degrees, m/s, %)
               ↓
  pix_command_manager             Priority order (highest → lowest):
  [command_arbitrator]            1. COLLISION_AVOIDANCE  → /pix/commands/collision_avoidance
                                  2. HUMAN_AVOIDANCE      → /pix/commands/human_avoidance
  Picks freshest top-priority     3. LANE_FOLLOWING       → /pix/commands/lane_following
  source, publishes winner        4. CRUISE_CONTROL       → /pix/commands/cruise_control
               ↓                  E-STOP: unconditional override (separate callback)
        /pix/raw_control_cmd
               ↓
  pix_safety_manager              Clamps: steer ±500°, speed 0–5 m/s, accel 0–3 m/s², brake 0–100%
  [safety_manager]                Watchdog: 300ms timeout → sends safe zero-speed command
               ↓
        /pix/control_cmd
               ↓
  pix_vehicle_interface_cpp       Pure encoder — 50Hz CAN loop, no arbitration, no clamping
               ↓
  pix_can_driver → SocketCAN → VCU (CAN4 @ 500 kbps)
```

**AEB signal flow:**
```
Camera → yolo_perception_node → /perception/obstacles → aeb_node
                                 [distance, lateral_offset]
aeb_node: if distance/speed < TTC threshold:
  publish_control_cmd(brake_en=True, brake_target=100%, emergency_stop=True)
  → /pix/commands/collision_avoidance  (Priority 1 — overrides straight_drive)
```

---

## Individual Actuator Tests

Run **Terminal 1** (`hw_framework.launch.py`) first, then in a new terminal:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# Test steering (vehicle does NOT move)
python3 scripts/actuator_test.py --mode hw --test steering

# Test braking (vehicle does NOT move)
python3 scripts/actuator_test.py --mode hw --test brake

# Test throttle — ⚠ VEHICLE WILL MOVE — ensure 15m clear space
python3 scripts/actuator_test.py --mode hw --test throttle

# Test gear cycling
python3 scripts/actuator_test.py --mode hw --test gear

# Test parking brake
python3 scripts/actuator_test.py --mode hw --test park

# Run ALL tests in sequence (interactive)
python3 scripts/actuator_test.py --mode hw --test full
```

**What to verify per test:**
| Test | `ros2 topic echo /pix/raw_control_cmd` | `candump can4` |
|---|---|---|
| steering | `steer_en: true`, `steer_target: ±200.0` | 0x102 frames at 50 Hz |
| brake | `brake_en: true`, `brake_target: 30.0` | 0x101 frames |
| throttle | `drive_en: true`, `speed_target: 1.5` | 0x100 frames |
| gear | `gear_en: true`, `gear_target: 4` | 0x103 frames |

---

## AEB + Straight Drive Test Sequence

1. **Pre-flight:** VCU in AUTO, e-stop green, CAN up (`candump can4 -n5` shows 0x5xx frames)
2. **Terminal 1:** `ros2 launch launch/hw_framework.launch.py`
3. **Terminal 2:** `ros2 launch pix_autonomy autonomy_test.launch.py`
4. **Verify driving:** `ros2 topic echo /pix/raw_control_cmd` → `drive_en: true, speed_target: 1.5`
5. **Check arbitrator:** `ros2 node list | grep arbitrator` → exactly one: `/command_arbitrator`
6. **AEB test:** walk in front of camera — terminal 1 logs `AEB ENGAGED!` — vehicle brakes
7. **AEB clear:** step away — logs `AEB cleared — resuming normal operation` — vehicle drives again

---

## Diagnostic Commands

```bash
# Is the pipeline alive?
ros2 topic hz /pix/raw_control_cmd        # should be ~50 Hz
ros2 topic hz /pix/control_cmd            # should be ~50 Hz

# What is the arbitrator publishing?
ros2 topic echo /pix/raw_control_cmd      # check drive_en, speed_target, gear_target

# Is there exactly one arbitrator? (Must be 1 — never 2)
ros2 node list | grep arbitrator

# Is YOLO publishing obstacles?
ros2 topic echo /perception/obstacles     # data: [distance, lateral_offset]

# CAN frame verification
candump can4 | grep " 100"               # throttle commands
candump can4 | grep " 102"               # steering commands
candump can4 | grep " 505"               # VCU mode (byte[4]=0x01 = AUTO)
```

---

## Writing a New Algorithm (30-second guide)

**Step 1:** Create your node inheriting `BaseAlgorithmInterface`:
```python
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class MyNode(BaseAlgorithmInterface):
    def __init__(self):
        super().__init__('my_node', '/pix/commands/cruise_control')  # must match YAML
        self.timer = self.create_timer(0.02, self.loop)

    def loop(self):
        self.publish_control_cmd(
            drive_en=True, speed_target=2.0, accel_target=1.0,
            steer_en=True, steer_target=0.0, steer_speed=150.0,
            gear_en=True,  gear_target=4,   # 4 = DRIVE
            park_en=True,  park_target=0,   # 0 = RELEASE
        )
```

**Step 2:** Register in `src/pix_command_manager/config/arbitrator_params.yaml` — add name + topic in priority order.

**Step 3:** `colcon build --symlink-install` — done.

---

## Field Units Reference

| Field | Unit | Max (safety manager) |
|---|---|---|
| `steer_target` | degrees (wheel) | ±500° |
| `steer_speed` | deg/s | 250°/s |
| `speed_target` | m/s | 5.0 m/s |
| `accel_target` | m/s² | 3.0 m/s² |
| `brake_target` | % | 100% |
| `gear_target` | int | 1=Park, 2=Rev, 3=Neutral, **4=Drive** |
| `park_target` | int | 0=Release, 1=Engage |

---

## CAN Frame Map

| CAN ID | Direction | Content |
|--------|-----------|---------|
| 0x100 | PC→VCU | Throttle command |
| 0x101 | PC→VCU | Brake command |
| 0x102 | PC→VCU | Steering command |
| 0x103 | PC→VCU | Gear command |
| 0x104 | PC→VCU | Park command |
| 0x500 | VCU→PC | Throttle report |
| 0x502 | VCU→PC | Steering report |
| 0x503 | VCU→PC | Gear report |
| 0x505 | VCU→PC | VCU mode (byte[4]=0x01 = AUTO) |
