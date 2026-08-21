# PIXKIT Autonomous Campus Shuttle — Full Development Guide
*Framework v9 · BITS Pilani · Last updated: 2026-07-03*

---

## Table of Contents
1. [Field Test Results — July 3, 2026](#1-field-test-results--july-3-2026)
2. [Braking Fix — Smooth Stop](#2-braking-fix--smooth-stop)
3. [Current System Status](#3-current-system-status)
4. [Framework Architecture](#4-framework-architecture)
5. [How to Add Any Algorithm](#5-how-to-add-any-algorithm)
6. [Path Planning Integration](#6-path-planning-integration)
7. [Tuning Parameters](#7-tuning-parameters)
8. [Sensor Integration Roadmap](#8-sensor-integration-roadmap)
9. [Next Steps — Phased Plan](#9-next-steps--phased-plan)
10. [Quick Reference](#10-quick-reference)

---

## 1. Field Test Results — July 3, 2026

### What was confirmed working ✅
| Test | Result |
|------|--------|
| Steering ±200° | ✅ Physical wheels turning |
| Brake 30% | ✅ Pressure confirmed |
| Gear shift N↔D↔R | ✅ (v9 checksum fix) |
| Park engage/release | ✅ |
| Throttle / forward motion | ✅ Vehicle moved! |
| Speed reached | **2.02 m/s (7.27 km/h)** |

### Issue found — Harsh braking
```
Brake command: 0% → 100% instantly
Speed at brake: 2.02 m/s
Time to stop:   0.60 s
Deceleration:   3.38 m/s²   ← HARSH (safe comfort limit = 1.5 m/s²)
```

**Fix in v9:** See Section 2 below.

---

## 2. Braking Fix — Smooth Stop

### What changed in `actuator_test.py` Step 5
```
OLD (harsh):  speed=0 + brake=100% simultaneously → 3.38 m/s²

NEW (smooth):
  Phase 5a: SpeedTarget=0, brake=OFF (3 s)
            → VCU motor braking decelerates naturally at ~1 m/s²
  Phase 5b: brake ramp 15% → 30% (2 s each step)
            → Final hold only, vehicle nearly stopped
  Step 6:   NEUTRAL + park, brake=50% (not 100%)

Result: ~1 m/s² decel  vs  old 3.38 m/s²
```

### Rule for your algorithms
```python
# SMOOTH STOP (for normal operation)
cmd.drive_en     = True
cmd.speed_target = 0.0          # Let VCU motor-brake first
cmd.brake_en     = False        # NO brake yet
cmd.brake_target = 0.0

# Wait until speed < 0.3 m/s, then:
cmd.brake_en     = True
cmd.brake_target = 30.0         # Gentle final hold

# EMERGENCY STOP (Ctrl+C only)
cmd.brake_en     = True
cmd.brake_target = 100.0        # Full immediate stop
cmd.emergency_stop = True
```

---

## 3. Current System Status

### What is fully working (v9)
| Component | Status | Notes |
|-----------|--------|-------|
| CAN TX (0x100–0x105) | ✅ | Correct per-message checksums |
| CAN RX (0x500–0x512) | ✅ | 50 Hz vehicle status |
| Steering control | ✅ | ±500° range |
| Brake control | ✅ | 0–100% |
| Gear shift N/D/R | ✅ | SUM checksum fixed |
| Park brake | ✅ | Via 0x104 |
| Throttle (Speed Drive) | ✅ | Up to 3 m/s safety cap |
| Safety manager | ✅ | Watchdog 300 ms |
| Command arbitrator | ✅ | Priority MUX |
| Modular algorithm launch | ✅ | Hot-swap capable |
| Smooth braking | ✅ (v9 fixed) | ~1 m/s² decel |

### Limits currently set
| Parameter | Value | File to change |
|-----------|-------|---------------|
| Max speed | 3.0 m/s | `launch/hw_framework.launch.py` |
| Max steer | 280° | `launch/hw_framework.launch.py` |
| Watchdog | 300 ms | `launch/hw_framework.launch.py` |
| Test speed | 1.5 m/s | `scripts/actuator_test.py` |

---

## 4. Framework Architecture

```
┌─────────────────────────────────────────────────────────┐
│              ALGORITHM LAYER (your code)                 │
│                                                         │
│  PathPlanning  YOLO   LaneFollow  ObstacleAvoid  ...    │
│       ↓          ↓        ↓            ↓                │
│  /pix/commands/cruise_control   (PixControlCmd msg)     │
└─────────────────────┬───────────────────────────────────┘
                      ↓ (ROS topic, 10+ Hz)
┌─────────────────────────────────────────────────────────┐
│              CORE INTERFACE (always running)             │
│                                                         │
│  Command Arbitrator → Safety Manager → CAN TX           │
│                                    ↓                    │
│                          SocketCAN (can4, 500 kbps)     │
│                                    ↕                    │
│                     VCU (Hooke2) ← CAN RX               │
│                          ↓                              │
│              /pix/vehicle_status (50 Hz)                │
└─────────────────────────────────────────────────────────┘
```

### Key topics
| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/pix/commands/cruise_control` | → Core | PixControlCmd | Algorithm commands in |
| `/pix/vehicle_status` | ← Core | PixVehicleStatus | Vehicle feedback out |
| `/pix/system_state` | ← Core | SystemState | STANDBY/AUTO/FAULT |
| `/pix/estop_trigger` | → Core | Bool | Trigger emergency stop |
| `/pix/estop_clear` | → Core | Bool | Clear estop latch |
| `/diagnostics` | ← Core | DiagnosticArray | Health monitoring |

---

## 5. How to Add Any Algorithm

### The interface contract
Any algorithm that publishes `PixControlCmd` to `/pix/commands/cruise_control` at ≥10 Hz will be automatically picked up by the arbitrator and actuated on the vehicle. That's it.

### Step-by-step template

**Step 1 — Create ROS 2 package**
```bash
cd src/algorithms/
ros2 pkg create my_algorithm --build-type ament_python \
    --dependencies rclpy pix_vehicle_msgs
```

**Step 2 — Write your node**
```python
# src/algorithms/my_algorithm/my_algorithm/my_algo_node.py
import rclpy
from rclpy.node import Node
from pix_vehicle_msgs.msg import PixControlCmd, PixVehicleStatus

class MyAlgorithm(Node):
    def __init__(self):
        super().__init__('my_algorithm')

        # Subscribe to vehicle feedback
        self.create_subscription(
            PixVehicleStatus, '/pix/vehicle_status', self._status_cb, 10)

        # Publish commands to the interface
        self.cmd_pub = self.create_publisher(
            PixControlCmd, '/pix/commands/cruise_control', 10)

        # Control loop at 20 Hz (must be > 3.3 Hz to avoid watchdog)
        self.create_timer(0.05, self._control_loop)

        self.speed  = 0.0
        self.gear   = 3
        self.mode   = 0

    def _status_cb(self, msg: PixVehicleStatus):
        self.speed = msg.vehicle_speed   # m/s
        self.gear  = msg.gear_actual     # 3=N, 4=D, 2=R
        self.mode  = msg.vehicle_mode    # 1=AUTO

    def _control_loop(self):
        cmd = PixControlCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # ── YOUR LOGIC HERE ──────────────────────────────────
        target_speed = 1.5    # m/s — from your planner
        steer_angle  = 0.0    # degrees — from your path tracker
        # ─────────────────────────────────────────────────────

        cmd.steer_en     = True
        cmd.steer_target = float(steer_angle)   # -500 to +500 deg
        cmd.steer_speed  = 100.0                # deg/s

        cmd.drive_en     = True
        cmd.speed_target = float(target_speed)  # m/s (capped at 3 m/s by safety_manager)
        cmd.accel_target = 1.0                  # m/s²

        cmd.gear_en      = True
        cmd.gear_target  = 4                    # 4=DRIVE

        cmd.brake_en     = False
        cmd.brake_target = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MyAlgorithm()
    rclpy.spin(node)
    rclpy.shutdown()
```

**Step 3 — Create launch file**
```python
# launch/algorithms/my_algorithm.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_algorithm',
            executable='my_algo_node',
            name='my_algorithm',
            output='screen',
        )
    ])
```

**Step 4 — Register in CMakeLists / setup.py**
```python
# In setup.py entry_points:
'console_scripts': [
    'my_algo_node = my_algorithm.my_algo_node:main',
],
```

**Step 5 — Run**
```bash
# Terminal 1 (already running)
ros2 launch launch/hw_framework.launch.py profile:=hardware

# Terminal 2 — your algorithm (hot-swap, no core restart needed)
ros2 launch launch/algorithms/my_algorithm.launch.py
```

### PixControlCmd field reference
```
steer_en:      bool   Enable steer DBW
steer_target:  float  Target angle, -500 to +500 degrees
steer_speed:   float  Steer rate, 0-250 deg/s

drive_en:      bool   Enable throttle DBW
speed_target:  float  Target speed in m/s (safety cap: 3.0 m/s)
accel_target:  float  Acceleration in m/s²

brake_en:      bool   Enable brake DBW
brake_target:  float  Brake pressure 0-100%

gear_en:       bool   Enable gear DBW
gear_target:   int    2=REVERSE, 3=NEUTRAL, 4=DRIVE

park_en:       bool   Enable park brake
park_target:   int    0=RELEASE, 1=ENGAGE

emergency_stop: bool  Override everything → full brake
```

---

## 6. Path Planning Integration

### Architecture for campus autonomous driving

```
GPS/RTK ──→ Localization node ──→ /pix/pose (geometry_msgs/PoseStamped)
LiDAR    ──→ Obstacle Detection ──→ /pix/obstacles
Camera   ──→ Lane Detection     ──→ /pix/lane_center

                ↓
         Global Planner         (gives waypoint route)
                ↓
         Local Planner          (gives immediate steering/speed)
                ↓
         /pix/commands/cruise_control
                ↓
         Core Interface → VCU → Vehicle moves
```

### Option A — Pure Pursuit (simplest, works now)

Pure pursuit tracks a list of GPS/map waypoints.

```python
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.waypoints = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),  # right turn
            (20.0, 10.0),
        ]
        self.lookahead = 3.0   # metres — tune this
        self.wheelbase = 1.5   # metres — measure on vehicle

        self.current_pose = None
        self.create_subscription(PoseStamped, '/pix/pose', self._pose_cb, 10)
        self.cmd_pub = self.create_publisher(PixControlCmd, '/pix/commands/cruise_control', 10)
        self.create_timer(0.05, self._loop)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _loop(self):
        if self.current_pose is None:
            return
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        yaw = self._get_yaw(self.current_pose.pose.orientation)

        # Find lookahead point
        target = self._lookahead_point(x, y, yaw)
        if target is None:
            return

        # Pure pursuit steering angle
        dx = target[0] - x
        dy = target[1] - y
        alpha = math.atan2(dy, dx) - yaw
        steer_rad = math.atan2(2 * self.wheelbase * math.sin(alpha), self.lookahead)
        steer_deg = math.degrees(steer_rad)  # convert to degrees for VCU

        cmd = PixControlCmd()
        cmd.steer_en     = True
        cmd.steer_target = max(-200.0, min(200.0, steer_deg * 10))  # VCU scale
        cmd.steer_speed  = 100.0
        cmd.drive_en     = True
        cmd.speed_target = 1.5   # constant speed — replace with velocity profile
        cmd.gear_en      = True
        cmd.gear_target  = 4
        self.cmd_pub.publish(cmd)
```

### Option B — Autoware integration (advanced)

Autoware already outputs `/control/command/control_cmd` (AckermannControlCommand).
Write a bridge node:

```python
from autoware_auto_control_msgs.msg import AckermannControlCommand

class AutowareBridge(Node):
    def __init__(self):
        super().__init__('autoware_bridge')
        self.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            self._aw_cb, 10)
        self.cmd_pub = self.create_publisher(
            PixControlCmd, '/pix/commands/cruise_control', 10)

    def _aw_cb(self, msg):
        cmd = PixControlCmd()
        # Autoware gives: steering_tire_angle (rad), speed (m/s)
        steer_rad = msg.lateral.steering_tire_angle
        steer_deg = math.degrees(steer_rad)

        cmd.steer_en     = True
        cmd.steer_target = steer_deg * 15.0  # tune scale factor for your vehicle
        cmd.steer_speed  = 150.0
        cmd.drive_en     = True
        cmd.speed_target = msg.longitudinal.speed
        cmd.gear_en      = True
        cmd.gear_target  = 4 if msg.longitudinal.speed >= 0 else 2
        cmd.brake_en     = msg.longitudinal.speed < 0.1
        cmd.brake_target = 30.0 if msg.longitudinal.speed < 0.1 else 0.0
        self.cmd_pub.publish(cmd)
```

### Campus route waypoints (example format)
```yaml
# config/campus_route.yaml
route:
  - name: "Main Gate"
    lat: 28.3614
    lon: 73.3120
    speed_limit: 1.5   # m/s
  - name: "Library"
    lat: 28.3618
    lon: 73.3125
    speed_limit: 1.0   # slow zone
  - name: "Hostel Block"
    lat: 28.3622
    lon: 73.3130
    speed_limit: 1.5
```

---

## 7. Tuning Parameters

### Safety Manager (edit `launch/hw_framework.launch.py`)
```python
params = {
    'max_speed':         3.0,   # m/s — start at 1.5, increase after testing
    'max_steer_angle': 280.0,   # degrees
    'max_steer_rate':  150.0,   # deg/s
    'watchdog_timeout':  0.3,   # seconds — reduce for tighter safety
    'max_brake':        100.0,  # % — keep at 100
}
```

### Throttle test (`scripts/actuator_test.py`)
```python
THROTTLE_TARGET = 1.5   # m/s — change for speed tests
THROTTLE_ACCEL  = 0.5   # m/s² — ramp rate
HOLD_SECONDS    = 3.0   # s — hold duration
```

### Steering scale factor (find empirically)
The VCU expects steer angle in degrees with an offset of 500.
- `steer_target = 0` → wheels centered
- `steer_target = +200` → right turn
- `steer_target = -200` → left turn

Map your algorithm's output (radians or normalized) to this:
```python
# If your algorithm outputs steering angle in radians:
steer_deg = math.degrees(steer_rad_from_planner)
cmd.steer_target = steer_deg * SCALE   # tune SCALE empirically

# Typical range for campus turns: ±50° to ±150°
# Full lock: ±280°
```

### Pure Pursuit lookahead tuning
```
Speed 1.0 m/s → lookahead = 2.0 m  (tight turns)
Speed 1.5 m/s → lookahead = 3.0 m  (normal)
Speed 2.0 m/s → lookahead = 4.0 m  (straight stretches)

Rule: lookahead = K × speed   where K ≈ 1.5–2.0
```

### Speed profile at junctions
```python
def get_speed_for_segment(dist_to_waypoint, is_turn):
    if dist_to_waypoint < 5.0 or is_turn:
        return 0.8   # slow before turns / stops
    return 1.5       # normal cruise
```

---

## 8. Sensor Integration Roadmap

### What you need for fully autonomous campus operation

| Sensor | Purpose | ROS Package |
|--------|---------|-------------|
| GPS/RTK (u-blox F9P) | Localization | `nmea_navsat_driver` |
| LiDAR (Ouster / Velodyne) | Obstacle detection, mapping | `ros2_ouster`, `ros2_velodyne` |
| IMU (Xsens / VectorNav) | Heading, roll/pitch | `imu_tools` |
| Camera (Realsense D435i) | Lane detection, person avoidance | `realsense2_camera` |
| Ultrasonic (front/rear) | Close-range obstacle | custom |

### Localization stack
```bash
# GPS-based (simplest for campus)
ros2 run nmea_navsat_driver nmea_topic_driver

# With map-based (LiDAR + SLAM)
ros2 launch slam_toolbox online_async_launch.py

# With Autoware (full autonomous stack)
ros2 launch autoware_launch autoware.launch.xml \
    map_path:=/maps/campus vehicle_model:=pixkit sensor_model:=sensing
```

### Obstacle detection integration
```python
# Your obstacle avoidance node subscribes to:
#   /scan (LaserScan) or /points (PointCloud2)
# and modifies speed/steer accordingly

class ObstacleAwareNode(Node):
    def __init__(self):
        self.obstacle_dist = 999.0
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _scan_cb(self, msg):
        # Find minimum distance in front 30° cone
        front = msg.ranges[len(msg.ranges)//2 - 15 : len(msg.ranges)//2 + 15]
        self.obstacle_dist = min(r for r in front if r > 0.1)

    def _control_loop(self):
        if self.obstacle_dist < 2.0:
            speed = 0.0             # stop
        elif self.obstacle_dist < 5.0:
            speed = 0.5             # slow
        else:
            speed = 1.5             # normal
        # ... publish cmd
```

---

## 9. Next Steps — Phased Plan

### Phase 1 — Complete hardware validation (NOW)
- [ ] Repeat throttle test with smooth stop fix (target: <1.5 m/s² decel)
- [ ] Test at 2.0 m/s (within 3 m/s safety cap)
- [ ] Measure actual steering angle vs VCU feedback (calibrate scale)
- [ ] Log and verify: park_actual=1 after every stop

**Commands:**
```bash
python3 scripts/actuator_test.py --mode hw --test throttle
# Watch vehicle_speed in feedback — confirm smooth ramp down
```

### Phase 2 — Manual waypoint driving (1-2 weeks)
- [ ] Mount GPS/RTK on vehicle
- [ ] Record campus route as GPS waypoints
- [ ] Implement Pure Pursuit node
- [ ] Test: vehicle follows straight path 20m
- [ ] Test: vehicle makes 90° turn at junction

```bash
# Record a route
ros2 bag record /pix/vehicle_status /fix -o campus_route_bag

# Replay for waypoint extraction
python3 scripts/extract_waypoints.py campus_route_bag
```

### Phase 3 — Obstacle awareness (2-4 weeks)
- [ ] Mount LiDAR (or use camera YOLO already done)
- [ ] Integrate obstacle distance → speed reduction
- [ ] Test: vehicle slows/stops for pedestrian
- [ ] Test: vehicle resumes after obstacle clears

### Phase 4 — Full campus route (1-2 months)
- [ ] Map the entire campus route
- [ ] Integrate full localization (GPS + IMU)
- [ ] Tune speed profiles per segment
- [ ] Add traffic-aware stops (e.g., at gates)
- [ ] Deploy with remote E-stop for safety observer

### Phase 5 — Multi-sensor fusion (advanced)
- [ ] Integrate Autoware as planning backend
- [ ] LiDAR-based SLAM for indoor/covered areas
- [ ] V2X communication for gate signals
- [ ] Fleet management for multiple shuttles

---

## 10. Quick Reference

### Every session
```bash
# CAN interface (every boot)
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000

# Launch core (Terminal 1 — keep running)
cd ~/Downloads/pix_control_framework
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch launch/hw_framework.launch.py profile:=hardware

# Run test (Terminal 2)
python3 scripts/actuator_test.py --mode hw --test throttle

# Monitor
ros2 topic echo /pix/vehicle_status
candump can4 | grep " 505 "   # VCU mode
```

### Emergency stop
```bash
# Ctrl+C in actuator_test terminal → 100% brake automatically
# OR from another terminal:
ros2 topic pub /pix/estop_trigger std_msgs/msg/Bool '{data: true}' --once
# Clear after:
ros2 topic pub /pix/estop_clear std_msgs/msg/Bool '{data: true}' --once
```

### Launch your algorithm
```bash
# After core is running:
ros2 launch launch/algorithms/yolo_avoidance.launch.py
# OR your custom:
ros2 run my_algorithm my_algo_node
```

### Verify gear frames (v9 checksum)
```bash
candump can4 | grep " 103 "
# NEUTRAL: 01 03 00 00 00 00 00 04  ← SUM cs=0x04 ✓
# DRIVE:   01 04 00 00 00 00 00 05  ← SUM cs=0x05 ✓
```

### Key file locations
| File | Purpose |
|------|---------|
| `src/pix_vehicle_interface/pix_vehicle_interface/can_tx.py` | CAN frame builders, checksums |
| `src/pix_vehicle_interface/pix_vehicle_interface/can_rx.py` | VCU feedback decoder |
| `launch/hw_framework.launch.py` | Core launch + safety limits |
| `launch/algorithms/` | Algorithm launch files |
| `scripts/actuator_test.py` | Commissioning tests |
| `DEPLOYMENT_GUIDE.md` | Hardware deployment steps |
| `AUTONOMOUS_CAMPUS_GUIDE.md` | This file — development guide |

---

## Summary Table — What Works, What's Next

| Capability | Status | Next action |
|------------|--------|-------------|
| CAN communication | ✅ | — |
| All actuators (steer/brake/gear/park/throttle) | ✅ | — |
| Smooth braking | ✅ v9 fixed | Test in field |
| Modular algorithm API | ✅ | Add your algorithms |
| YOLO person avoidance | ✅ built | Test with moving vehicle |
| GPS localization | ❌ | Mount GPS, run Phase 2 |
| Pure pursuit path following | ❌ | Implement after GPS |
| LiDAR obstacle detection | ❌ | Mount sensor, Phase 3 |
| Autoware integration | ❌ | Phase 5 (advanced) |
| Campus route mapping | ❌ | Phase 2 |
