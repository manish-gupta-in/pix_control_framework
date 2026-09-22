# PIX Autonomy Architecture Guide (v19)

This guide explains the industry-standard **Sense → Plan → Act** architecture implemented in the
`pix_autonomy` ROS2 package and how to integrate new algorithms into the PIX Control Framework.

---

## 1. The Autonomous Flow (Sense → Plan → Act)

In an industry-level system, sensors (cameras, LiDAR) **never** send steering commands directly to
the wheels. Data flows through specialized layers before reaching the VCU:

```mermaid
graph TD
    subgraph SENSE [Perception Layer]
        YOLO(YOLO Node) -->|pub: /perception/obstacles| O[Obstacle Array]
        GNSS(GNSS Sensor) -->|pub: /localization/pose| P[Vehicle Pose X,Y,Yaw]
    end

    subgraph PLAN [Decision & Planning Layer]
        O --> AEB(AEB Node)
        O --> LAT(Lateral Avoidance Node)
        P --> MPC(MPC Planner Node)
        P --> WP(GNSS Waypoint Node)
        X[Straight Drive Node] --> STRAIGHT[/pix/commands/lane_following]
    end

    subgraph ACT [Single Arbitration Pipeline]
        AEB -->|pub: /pix/commands/collision_avoidance| ARB
        LAT -->|pub: /pix/commands/human_avoidance| ARB
        WP -->|pub: /pix/commands/lane_following| ARB
        MPC -->|pub: /pix/commands/cruise_control| ARB
        STRAIGHT --> ARB

        ARB(pix_command_manager — Real Arbitrator) -->|pub: /pix/raw_control_cmd| SAFE(Safety Manager)
        SAFE -->|pub: /pix/control_cmd| VCU[pix_vehicle_interface_cpp → CAN → VCU]
    end
```

### The Roles

| Layer | Node | What it does |
|---|---|---|
| **Perception** | `yolo_perception_node` | Identifies *what* and *where* — outputs distances on `/perception/obstacles` |
| **Planning** | `aeb_node`, `straight_drive_node`, etc. | Maps perception → desired speed/steering; publishes to a `/pix/commands/<name>` topic |
| **Arbitration** | `pix_command_manager` (command_arbitrator) | Priority MUX — exactly **one** publisher on `/pix/raw_control_cmd` |
| **Safety** | `pix_safety_manager` | Hard-clamps, rate-limits, watchdog; gates into `/pix/control_cmd` |
| **Interface** | `pix_vehicle_interface_cpp` | Pure CAN encoder — no arbitration, no clamping |

> ⚠ **There is exactly ONE arbitrator in this system: `pix_command_manager/command_arbitrator`.
> Never run a second arbitrator node alongside it. Running `control_arbitrator_node` from any
> external package is prohibited — that file has been removed from the codebase.**

---

## 2. Package Overview (`pix_autonomy`)

The `pix_autonomy` package at `src/pix_autonomy` contains algorithm nodes only. It has **no
arbitrator**. All algorithm output goes to a registered `/pix/commands/<name>` topic which the
real arbitrator (in `pix_command_manager`) picks up.

### `yolo_perception_node.py` (The Eyes)
- Subscribes to camera feed; runs YOLO inference.
- Publishes an array of `[distance, normalized_offset]` on `/perception/obstacles`.

### `aeb_node.py` (The Reflexes)
- Subscribes to `/perception/obstacles` and vehicle speed.
- Calculates **Time-To-Collision (TTC)**. If TTC < threshold → publishes full-brake command.
- **Output topic:** `/pix/commands/collision_avoidance` (Priority 2 — overrides normal driving)

### `lateral_avoidance_node.py` (The Swerve)
- Subscribes to `/perception/obstacles`.
- Proportional lateral steering away from obstacles, with hold-time and ramp smoothing.
- **Output topic:** `/pix/commands/human_avoidance` (Priority 3)

### `straight_drive_node.py` (The Driver — AEB Test)
- Commands constant forward speed at 0° steering angle.
- **Output topic:** `/pix/commands/lane_following` (Priority 5 — lowest; safely overridden by AEB)

### `gnss_waypoint_follower.py` (The Basic Brain)
- Reads X,Y waypoints; Pure Pursuit steering.
- **Output topic:** `/pix/commands/lane_following` (Priority 5)

### `mpc_planner_node.py` (The Advanced Brain)
- Boilerplate Model Predictive Controller.
- **Output topic:** `/pix/commands/cruise_control` (Priority 5)

---

## 3. How to Add a NEW Algorithm (Step-by-Step)

You only need **two steps** to add a new algorithm. You do **not** touch any arbitrator code.

### Step 1: Create the node

```python
#!/usr/bin/env python3
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class LKANode(BaseAlgorithmInterface):
    def __init__(self):
        # The topic name is the only coupling to the arbitrator.
        # It must match a topic registered in arbitrator_params.yaml.
        super().__init__('lka_node', '/pix/commands/my_new_algorithm')
        self.timer = self.create_timer(0.02, self.control_loop)

    def control_loop(self):
        self.publish_control_cmd(
            drive_en=True, speed_target=2.0, accel_target=1.0,
            steer_en=True, steer_target=calculated_steer, steer_speed=150.0,
            brake_en=False, brake_target=0.0,
            gear_en=True, gear_target=4,   # 4 = DRIVE
            park_en=True, park_target=0    # 0 = RELEASE
        )

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = LKANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Step 2: Register in `arbitrator_params.yaml`

Edit `src/pix_command_manager/config/arbitrator_params.yaml`. Add your name + topic in the
priority order you want (lower index = higher priority):

```yaml
/**:
  ros__parameters:
    active_timeout: 0.4
    priority_sources:
      - "COLLISION_AVOIDANCE"
      - "HUMAN_AVOIDANCE"
      - "MY_NEW_ALGORITHM"        # ← add here
      - "LANE_FOLLOWING"
      - "CRUISE_CONTROL"
    priority_topics:
      - "/pix/commands/collision_avoidance"
      - "/pix/commands/human_avoidance"
      - "/pix/commands/my_new_algorithm"   # ← add here
      - "/pix/commands/lane_following"
      - "/pix/commands/cruise_control"
```

**That is all.** No arbitrator code changes required. Rebuild and relaunch.

### Step 3: Register in `setup.py`

```python
'console_scripts': [
    'lka_node = pix_autonomy.lka_node:main',
    ...
],
```

---

## 4. Complete Minimal Algorithm Example

This is the full, copy-pasteable pattern for a brand-new algorithm node.
Copy this file, rename the class and topic, and register the topic in
`arbitrator_params.yaml`. Nothing else needs to change.

```python
#!/usr/bin/env python3
"""
my_speed_hold_node.py — example algorithm for PIX Control Framework.

Subscribes: /pix/vehicle_status (via BaseAlgorithmInterface)
Publishes:  /pix/commands/cruise_control  (registered in arbitrator_params.yaml)

To run:
  ros2 run <your_package> my_speed_hold_node
"""
import rclpy
from std_msgs.msg import Float32
from pix_algorithm_api.base_algorithm_interface import BaseAlgorithmInterface

class SpeedHoldNode(BaseAlgorithmInterface):
    """
    Maintains a constant forward speed.
    Priority slot: CRUISE_CONTROL (lowest — overridden by AEB and avoidance).
    """
    def __init__(self):
        # Topic MUST match an entry in arbitrator_params.yaml priority_topics
        super().__init__('speed_hold_node', '/pix/commands/cruise_control')

        self.declare_parameter('target_speed', 2.0)   # m/s
        self.target_speed = self.get_parameter('target_speed').value

        # Example: subscribe to a sensor topic
        self.create_subscription(
            Float32, '/sensors/speed_setpoint', self.setpoint_callback, 10)

        # Control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f'SpeedHoldNode ready. target_speed={self.target_speed} m/s')

    def setpoint_callback(self, msg: Float32):
        """Update target speed from an external source."""
        self.target_speed = float(msg.data)

    def control_loop(self):
        status = self.get_vehicle_status()  # latest PixVehicleStatus (thread-safe)

        # Publish a PixControlCmd — degrees + m/s + percent, no unit conversion needed
        self.publish_control_cmd(
            drive_en=True,  speed_target=self.target_speed,  accel_target=1.0,
            steer_en=True,  steer_target=0.0,                steer_speed=150.0,
            brake_en=False, brake_target=0.0,
            gear_en=True,   gear_target=4,   # 4 = DRIVE
            park_en=True,   park_target=0,   # 0 = RELEASE
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpeedHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Matching `arbitrator_params.yaml` entry (already present for `CRUISE_CONTROL`):**
```yaml
priority_sources:
  - "CRUISE_CONTROL"          # already in the list
priority_topics:
  - "/pix/commands/cruise_control"   # already in the list
```
If you are adding a brand-new slot, append a new name+topic pair and rebuild.

---

## 5. Building and Running

```bash
# Build from workspace root
colcon build --symlink-install
source install/setup.bash
```

**Running the AEB + Straight-Line Test:**

```bash
# Terminal 1: Core hardware pipeline (CAN + arbitrator + safety)
ros2 launch launch/hw_framework.launch.py

# Terminal 2: Autonomy algorithms (YOLO + AEB + Straight Drive)
ros2 launch pix_autonomy autonomy_test.launch.py
```

That is exactly **two terminals** — the arbitrator is already running inside Terminal 1
via `pix_command_manager`. Never run a second arbitrator.
