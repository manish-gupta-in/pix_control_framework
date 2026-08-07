# PIXKIT Control Framework — Complete Reference Guide
**Version 2.0 | 13 Packages | 134 Tests — All PASS ✓**

---

## 1. Directory Structure (v2.0 — Final)

```
pix_control_framework/
├── launch/
│   ├── hw_framework.launch.py       ← Hardware: 9 nodes (CAN TX/RX, arbitrator, safety,
│   │                                              state manager, diagnostics, logger, config, YOLO)
│   ├── sim_framework.launch.py      ← Simulation: same minus CAN, plus simulator + RViz
│   └── sim_config.rviz              ← RViz display config
│
├── scripts/
│   └── actuator_test.py             ← ★ Manual actuator commissioning tool
│
└── src/
    ├── pix_vehicle_msgs/            ← ROS2 message definitions (CMake package)
    │   └── msg/
    │       ├── PixControlCmd.msg    ← All actuator commands + gear/park constants
    │       ├── PixVehicleStatus.msg ← Full VCU feedback + gear/park/battery constants
    │       └── PixSystemState.msg   ← NEW: system state, fault count, active algorithm
    │
    ├── pix_vehicle_interface/       ← CAN driver (SocketCAN ↔ ROS2)
    │   ├── config/
    │   │   ├── hook2_AD.dbc         ← PIX VCU DBC database (DO NOT MODIFY)
    │   │   ├── can_tx_params.yaml   ← ★ Tune: interface name, rate, enable flag
    │   │   └── can_rx_params.yaml   ← ★ Tune: interface name, rate
    │   ├── pix_vehicle_interface/
    │   │   ├── can_tx.py            ← Cyclically encodes 6 command frames → SocketCAN
    │   │   ├── can_rx.py            ← Reads report frames → /pix/vehicle_status
    │   │   ├── dbc_encoder.py       ← cantools wrapper + PIX XOR checksum (bytes 0–6 → byte 7)
    │   │   └── dbc_decoder.py       ← cantools wrapper for decoding report frames
    │   └── test/
    │       └── test_dbc_codec.py    ← 19 tests: encoding, frame IDs, roundtrip
    │
    ├── pix_safety_manager/          ← Safety envelope + rate limiter + watchdog
    │   ├── config/
    │   │   └── safety_params.yaml   ← ★ Tune: steer limits, speed limits, watchdog timeout
    │   ├── pix_safety_manager/
    │   │   └── safety_manager_node.py
    │   └── test/
    │       └── test_safety_logic.py ← 28 tests: clamp, rate limit, E-stop conditions
    │
    ├── pix_command_manager/         ← Priority-based command arbitration
    │   ├── config/
    │   │   └── arbitrator_params.yaml ← ★ Tune: algorithm timeout window
    │   ├── pix_command_manager/
    │   │   └── command_arbitrator.py  ← Priority: ESTOP > COLLISION > HUMAN > LANE > CRUISE
    │   └── test/
    │       └── test_arbitration_logic.py ← 11 tests: priority, timeout, standby fallback
    │
    ├── pix_state_manager/           ← NEW: System state machine
    │   ├── config/
    │   │   └── state_manager_params.yaml ← ★ Tune: timeouts, recovery delay
    │   ├── pix_state_manager/
    │   │   └── system_state_manager_node.py ← MANUAL/STANDBY/AUTONOMOUS/FAULT/ESTOP
    │   └── test/
    │       └── test_state_machine.py ← 18 tests: all transitions, fault handling, E-stop latch
    │
    ├── pix_diagnostics/             ← NEW: Health monitoring → /diagnostics
    │   ├── config/
    │   │   └── diagnostics_params.yaml ← ★ Tune: timeouts, battery thresholds
    │   ├── pix_diagnostics/
    │   │   └── diagnostics_node.py   ← 6 channels: CAN RX/TX, VCU faults, watchdog, battery, state
    │   └── test/
    │       └── test_diagnostics_logic.py ← 29 tests: all channel logic
    │
    ├── pix_logger/                  ← NEW: CSV logging framework
    │   ├── config/
    │   │   └── logger_params.yaml   ← ★ Toggle per-topic logging, flush interval
    │   ├── pix_logger/
    │   │   └── logger_node.py       ← Writes ~/pix_logs/<session>/*.csv at 50 Hz
    │   └── test/
    │       └── test_logger_config.py ← 17 tests: CSV structure, profile validation
    │
    ├── pix_config_manager/          ← NEW: YAML profile manager
    │   ├── profiles/
    │   │   ├── simulation.yaml      ← vcan0, relaxed limits for dev
    │   │   ├── hardware.yaml        ← can4, conservative limits for deployment
    │   │   └── tuning.yaml          ← can4, intermediate for field tuning
    │   └── pix_config_manager/
    │       └── config_manager_node.py ← Loads profile, publishes /pix/config/active_profile
    │
    ├── pix_algorithm_api/           ← Base class for all algorithm nodes
    │   └── pix_algorithm_api/
    │       └── base_algorithm_interface.py ← publish_control_cmd() helper
    │
    ├── pix_simulator/               ← 50Hz kinematic bicycle model + RViz output
    │   ├── config/
    │   │   └── simulator_params.yaml ← ★ Tune: wheelbase, max decel, friction
    │   └── test/
    │       └── test_kinematics.py   ← 17 tests: speed/steer/position/yaw integration
    │
    ├── lane_following/              ← Mock lane follower (priority 3)
    ├── object_tracking/             ← Mock object tracker (priority 1)
    └── yolo_person_avoidance/       ← YOLO lateral avoidance (priority 2)
        └── config/
            └── yolo_avoidance_params.yaml ← ★ Tune: model, gain, ramp, hold_frames
```

---

## 2. Topic Map

| Topic | Message Type | Publisher → Subscriber |
|---|---|---|
| `/pix/vehicle_status` | `PixVehicleStatus` | `can_rx` → all nodes |
| `/pix/control_cmd` | `PixControlCmd` | `safety_manager` → `can_tx` |
| `/pix/raw_control_cmd` | `PixControlCmd` | `command_arbitrator` → `safety_manager` |
| `/pix/commands/cruise_control` | `PixControlCmd` | actuator_test → arbitrator |
| `/pix/commands/lane_following` | `PixControlCmd` | `lane_following` → arbitrator |
| `/pix/commands/yolo_avoidance` | `PixControlCmd` | `yolo_avoidance` → arbitrator |
| `/pix/system_state` | `PixSystemState` | `state_manager` → all |
| `/pix/estop_trigger` | `Bool` | any → `state_manager` |
| `/pix/estop_clear` | `Bool` | operator → `state_manager` |
| `/diagnostics` | `DiagnosticArray` | `pix_diagnostics` → rqt/logging |
| `/pix/config/active_profile` | `String` (JSON) | `config_manager` → all |

---

## 3. System State Machine

```
Power ON
   │
   ▼
MANUAL (0) ──── DBW engaged ────────────► STANDBY (1)
   ▲                                           │
   │                                    Algorithm active
   │                                           │
   │                                           ▼
   │                                    AUTONOMOUS (2)
   │                                           │
   │         Any state (except ESTOP)          │
   │◄──── VCU fault detected ─────────────────┘
   │
   ▼
FAULT (3) ──── fault clears + 2s ───────► STANDBY (1)
   │
   │  (only via /pix/estop_trigger)
   ▼
ESTOP (4) ──── /pix/estop_clear=True ───► MANUAL (0)
```

**Trigger E-stop:**
```bash
ros2 topic pub /pix/estop_trigger std_msgs/msg/Bool "data: true" --once
```
**Clear E-stop (manual operator action required first):**
```bash
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool "data: true" --once
```

---

## 4. Unit Test Suite

**All 134 tests pass. No ROS2 daemon required.**

```bash
cd ~/Desktop/Manish/Custom_Interface_study/pix_control_framework
source /opt/ros/humble/setup.bash && source install/setup.bash

python3 -m pytest \
  src/pix_state_manager/test/test_state_machine.py \
  src/pix_diagnostics/test/test_diagnostics_logic.py \
  src/pix_logger/test/test_logger_config.py \
  src/pix_command_manager/test/test_arbitration_logic.py \
  src/pix_safety_manager/test/test_safety_logic.py \
  src/pix_simulator/test/test_kinematics.py \
  src/pix_vehicle_interface/test/test_dbc_codec.py \
  -v
```

| Test File | Tests | What It Covers |
|---|---|---|
| `test_state_machine.py` | 18 | All state transitions, fault latch, E-stop latch |
| `test_diagnostics_logic.py` | 29 | CAN age checks, VCU faults, battery thresholds, watchdog |
| `test_logger_config.py` | 17 | CSV structure, profile YAML existence and correctness |
| `test_arbitration_logic.py` | 11 | Priority order, timeout, standby fallback |
| `test_safety_logic.py` | 28 | Steer clamp, rate limiter, all E-stop conditions |
| `test_kinematics.py` | 17 | Speed, braking, E-stop, steering, position, yaw |
| `test_dbc_codec.py` | 19 | All 6 command encodings, 7 report IDs, roundtrip |

---

## 5. Config Tuning Reference

### `safety_params.yaml` (Hardware values)
| Parameter | HW Default | SIM Default | Effect |
|---|---|---|---|
| `max_steer_angle` | `280.0°` | `350.0°` | Hard cap on steering output |
| `max_steer_rate` | `150.0°/s` | `300.0°/s` | Max per-tick change — prevents jerks |
| `max_speed` | `3.0 m/s` | `5.0 m/s` | Speed cap passed to VCU |
| `max_accel` | `1.0 m/s²` | `3.0 m/s²` | Acceleration cap |
| `watchdog_timeout` | `0.3 s` | `0.6 s` | E-stop if no algorithm heartbeat |

### `state_manager_params.yaml`
| Parameter | Default | Effect |
|---|---|---|
| `algorithm_timeout` | `0.5 s` | Time without command before state drops to STANDBY |
| `fault_clear_delay` | `2.0 s` | Delay before recovering from FAULT → STANDBY |

### `diagnostics_params.yaml`
| Parameter | Default | Effect |
|---|---|---|
| `can_rx_timeout` | `0.5 s` | Age threshold for CAN RX WARN |
| `battery_warn_v` | `46.0 V` | Battery voltage warning threshold |
| `battery_error_v` | `42.0 V` | Battery voltage error threshold |
| `battery_warn_soc` | `20.0 %` | SOC warning threshold |

### `yolo_avoidance_params.yaml`
| Parameter | Default | Effect |
|---|---|---|
| `confidence_threshold` | `0.40` | Lower = more detections, higher = fewer false positives |
| `gain` | `300.0` | Avoidance steering magnitude per normalized pixel offset |
| `max_avoidance` | `500.0°` | Full avoidance steer lock (reduce for gentler arc) |
| `ramp_rate` | `200.0°/s` | Max steer change per tick |
| `hold_frames` | `15` | Hold command after person disappears (frames at ~30fps) |
| `target_speed` | `2.0 m/s` | Speed cap during active avoidance |

---

## 6. Build and Deploy

### Development Machine — Build Once
```bash
cd ~/Desktop/Manish/Custom_Interface_study/pix_control_framework
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Simulation (No Vehicle Needed)
```bash
ros2 launch launch/sim_framework.launch.py profile:=simulation
```

### Hardware Deployment
```bash
# Pre-flight: bring CAN up
sudo ip link set can4 up type can bitrate 500000
candump can4 -n 10     # verify: must see 0x500, 0x501, 0x502, 0x505, 0x512

# Launch all 9 nodes
ros2 launch launch/hw_framework.launch.py profile:=hardware
```

---

## 7. Vehicle Deployment — Full Sequence

### Step 1: Package & Transfer (Dev Machine)
```bash
# The compressed package is ready at:
~/Desktop/Manish/Custom_Interface_study/pix_control_framework_v2.tar.gz  # 5.8 MB

# Transfer via SCP:
scp pix_control_framework_v2.tar.gz sysadmin@<VEHICLE_IP>:~/

# Or via USB — copy file to USB, then on vehicle:
cp /media/<USB>/pix_control_framework_v2.tar.gz ~/
```

### Step 2: Extract & Build (Vehicle Computer)
```bash
cd ~
tar -xzvf pix_control_framework_v2.tar.gz
cd pix_control_framework
source /opt/ros/humble/setup.bash
colcon build --symlink-install    # expects: 13 packages finished
source install/setup.bash
```

### Step 3: Pre-flight
```bash
sudo ip link set can4 up type can bitrate 500000
candump can4 -n 20    # confirm VCU frames streaming
```

### Step 4: Launch
```bash
ros2 launch launch/hw_framework.launch.py profile:=hardware
```

### Step 5: Verify in Separate Terminals
```bash
# Terminal 2 — VCU feedback:
ros2 topic echo /pix/vehicle_status

# Terminal 3 — System state (should show state: 0 = MANUAL initially):
ros2 topic echo /pix/system_state

# Terminal 4 — Health check (all channels should be OK or WARN, not ERROR):
ros2 topic echo /diagnostics
```

### Step 6: Actuator Commissioning (STRICT ORDER)
```bash
# ALWAYS in this order — never skip:
python3 scripts/actuator_test.py --mode hw --test brake      # 1st — validates stop authority
python3 scripts/actuator_test.py --mode hw --test steering   # 2nd — wheels turn L/R
python3 scripts/actuator_test.py --mode hw --test gear       # 3rd — P→N→D→N→P
python3 scripts/actuator_test.py --mode hw --test park       # 4th — park release/engage
python3 scripts/actuator_test.py --mode hw --test throttle   # 5th — ⚠ VEHICLE MOVES ~5m
```

---

## 8. Monitoring & Diagnostics

```bash
# Check which algorithm is controlling:
ros2 topic echo /pix/system_state   # active_algorithm field

# Check what safety layer passes to VCU:
ros2 topic echo /pix/control_cmd

# View all CSV logs from the session:
ls ~/pix_logs/
head ~/pix_logs/<session>/vehicle_state.csv

# View active config profile:
ros2 topic echo /pix/config/active_profile
```

**CSV Files Written Per Session** (`~/pix_logs/<YYYYMMDD_HHMMSS>/`):
| File | Contents |
|---|---|
| `vehicle_state.csv` | Full VCU feedback at 50 Hz |
| `control_cmd.csv` | Safety-filtered command output |
| `raw_cmd.csv` | Pre-safety arbitrated command |
| `system_state.csv` | State transitions with timestamps |

---

## 9. Emergency Recovery

**Trigger E-stop from any terminal:**
```bash
ros2 topic pub /pix/estop_trigger std_msgs/msg/Bool "data: true" --once
```

**Clear E-stop (only after confirming area is safe):**
```bash
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool "data: true" --once
```

**Hard restart (if E-stop latch cannot be cleared):**
```bash
Ctrl+C   # in the launch terminal
ros2 launch launch/hw_framework.launch.py profile:=hardware
```

---

## 10. CAN Signal Reference (Verified vs Hooke2.0 Matrix)

| Message | CAN ID | Dir | Key Signals |
|---|---|---|---|
| `Steer_Control` | 0x100 | TX | `Steer_Angle` (±500°, offset –500), `Steer_AngleSpeed` (0–250°/s) |
| `Brake_Control` | 0x101 | TX | `Brake_En`, `Brake_PedalTarget` (0–100%) |
| `Throttle_Control` | 0x102 | TX | `Drive_En`, `ThrottleAcc`, `ThrottlePedalTarget` (0–100%) |
| `Park_Control` | 0x103 | TX | `Park_En`, `Park_Target` (0=Release, 1=Trigger) |
| `Gear_Control` | 0x104 | TX | `Gear_En`, `Gear_Target` (1=P, 2=R, 3=N, 4=D) |
| `VCU_Control` | 0x105 | TX | `VCU_DrivingMode_Cmd` (1=Auto, 2=Manual) |
| `Throttle_Report` | 0x500 | RX | `Drive_EnState`, `ThrottlePedalActual` |
| `Brake_Report` | 0x501 | RX | `Brake_EnState`, `Brake_PedalActual` |
| `Steer_Report` | 0x502 | RX | `Steer_EnState`, `Steer_AngleActual` |
| `Park_Report` | 0x503 | RX | `Park_EnState`, `Park_Actual` |
| `Gear_Report` | 0x504 | RX | `Gear_EnState`, `Gear_Actual` |
| `VCU_Report` | 0x505 | RX | `Vehicle_Speed`, `Vehicle_Mode` |
| `BMS_Report` | 0x512 | RX | `Battery_Voltage`, `Battery_SOC`, `Battery_Current` |

> Checksum: XOR of bytes 0–6 → byte 7. Computed automatically by `DBCEncoder`.

---

## 11. Known Issues Fixed in v2.0

| Issue | Root Cause | Fix |
|---|---|---|
| `(no feedback received yet)` in actuator_test | `can_rx` not running | hw_framework now always includes `can_rx` |
| Commands not reaching VCU (no 0x100–0x105 in candump) | `can_tx` missing from old launch file | hw_framework now always includes `can_tx` |
| Arbitrator spam `NONE → STANDBY/NONE` every 20 ms | Sentinel comparison bug | Fixed: stable label, only logs on real state change |
| `Steer_AngleSpeed` encoding overflow (>255) | 8-bit DBC field overflow | `min(speed, 250)` clamp in `can_tx.py` |

---

*Framework Version 2.0 | 13 packages | 134 tests all pass | Deployment archive: pix_control_framework_v2.tar.gz*
