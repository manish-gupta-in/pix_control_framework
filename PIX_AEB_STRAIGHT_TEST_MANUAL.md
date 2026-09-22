# PIX AEB & Straight Line Test Manual (v20)

This manual provides step-by-step instructions for performing a **50-meter straight line test**
with the **Autonomous Emergency Braking (AEB)** system active.

In this test, the vehicle drives straight forward at a constant speed. If a person steps into the
camera view, the AEB node publishes a full-brake override command to the **single real arbitrator**
(`pix_command_manager`), which immediately pre-empts the straight-drive command, and the vehicle
brakes to a full stop.

> ⚠ **IMPORTANT — Single Arbitrator Rule**
> There is exactly **one** arbitrator in this system: `pix_command_manager/command_arbitrator`,
> started automatically by `launch/hw_framework.launch.py`.
> **Do NOT run `control_arbitrator_node` or any second arbitrator node.** If two arbitrators
> publish to `/pix/raw_control_cmd` simultaneously, the vehicle will randomly flip between
> STANDBY and AUTONOMOUS mode and will not actuate reliably.

---

## 1. Safety Prerequisites

- ⚠ Ensure you have at least **50 metres of completely clear, flat ground** ahead of the vehicle.
- ⚠ Ensure the **Hardware E-Stop button** (on the vehicle or RC controller) is within reach at all
  times.
- ⚠ Ensure the VCU physical switch is set to **AUTO** (not STANDBY) before launching.

---

## 2. Setting Up the Vehicle

1. Clone the framework repository (or extract `pix_control_framework_v20.zip`) on the vehicle computer:
   ```bash
   git clone git@github.com:manish-gupta-in/pix_control_framework.git
   cd pix_control_framework
   ```
2. Build the workspace (only needs to be done once):
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Bring up the CAN interface:
   ```bash
   sudo ip link set can4 up type can bitrate 500000
   sudo ip link set can4 txqueuelen 1000
   ```
4. Verify VCU frames are coming in:
   ```bash
   candump can4 -n 10
   # You should see frames from the VCU at 50 Hz
   ```

---

## 3. Running the Test — Exactly Two Terminals

In every terminal, source the workspace first:
```bash
cd ~/pix_control_framework
source install/setup.bash
```

### Terminal 1: Core Framework (Hardware Interface + Arbitrator)

```bash
ros2 launch launch/hw_framework.launch.py
```

This starts:
- `pix_can_driver` — SocketCAN bridge
- `pix_vehicle_interface_cpp` — 50 Hz CAN encoder
- **`command_arbitrator`** — the single priority MUX
- `safety_manager` — hard-clamps and watchdog
- `system_state_manager`, `diagnostics_node`, `logger_node`, `config_manager`

Verify: you see CAN traffic and no `ERROR` messages. The vehicle should be in STANDBY.

### Terminal 2: Autonomy Algorithms (YOLO + AEB + Straight Drive)

```bash
ros2 launch pix_autonomy autonomy_test.launch.py
```

This starts:
- `yolo_perception_node` — camera + YOLO inference → `/perception/obstacles`
- `aeb_node` — TTC watchdog → `/pix/commands/collision_avoidance` (Priority 2)
- `straight_drive_node` — constant speed forward → `/pix/commands/lane_following` (Priority 5)

> ⚠ **The vehicle will start moving forward as soon as Terminal 2 launches.**
> Ensure the area is clear before running this command.

---

## 4. How to Tune Parameters (Speed, AEB Distance, etc.)

You do **NOT** need to rebuild the workspace to change speeds or AEB settings. Edit:

```
src/pix_autonomy/config/autonomy_params.yaml
```

Relevant settings:
- **`speed: 1.5`** (under `straight_drive_planner`) — constant driving speed in m/s.
- **`ttc_threshold: 2.0`** (under `aeb_node`) — seconds; increase to brake earlier, decrease to
  brake later.

After saving, re-run Terminal 2 to pick up the new parameters.

---

## 5. The Test Procedure

1. The vehicle drives straight at the configured speed (default `1.5 m/s`).
2. Have a person safely walk into the camera frame from the side (at least 5–10 m ahead).
3. **Expected behaviour:**
   - `yolo_perception_node` detects the person and publishes `[distance, offset]` on
     `/perception/obstacles`.
   - `aeb_node` calculates TTC. If TTC < 2.0 s, it publishes a full-brake command to
     `/pix/commands/collision_avoidance`.
   - The **real arbitrator** (`pix_command_manager`) pre-empts `lane_following` with
     `collision_avoidance` (higher priority) and outputs the brake command to
     `/pix/raw_control_cmd`.
   - `safety_manager` passes it through to `/pix/control_cmd` → VCU.
   - The vehicle brakes immediately. The Terminal 1 log prints:
     ```
     [WARN] PREEMPTION EVENT: [LANE_FOLLOWING] overridden by [COLLISION_AVOIDANCE]
     ```
   - Terminal 2 prints: `[ERROR] AEB ENGAGED! TTC: x.xxs, Dist: y.yym`

4. Once the person clears the frame, `aeb_node` stops publishing. After the `active_timeout`
   (0.4 s) the arbitrator falls back to `lane_following` and the vehicle resumes driving.

---

## 6. If the Vehicle Doesn't Move — Troubleshooting

Work through these checks in order:

### Check 1: Is the arbitrator outputting commands?
```bash
ros2 topic echo /pix/raw_control_cmd
```
You should see `drive_en: true` at 50 Hz. If the topic is silent, the arbitrator isn't receiving
any algorithm commands — check Terminal 2 for errors.

### Check 2: Is the safety manager passing commands through?
```bash
ros2 topic echo /pix/control_cmd
```
If `/pix/raw_control_cmd` is alive but `/pix/control_cmd` is not, the safety manager has triggered
its watchdog or E-stop. Check Terminal 1 for `[WARN] Watchdog` messages.

### Check 3: Is the VCU receiving CAN frames?
```bash
candump can4 | grep 100
```
You should see CAN frames on `0x100` (throttle command) at ~50 Hz. If not, the vehicle interface
node has a CAN socket issue.

### Check 4: Is the VCU in AUTO mode?
```bash
candump can4 | grep 505
```
Byte[4] must be `01` (Auto). `03` means STANDBY — the VCU will ignore all gear/throttle commands.
- Disengage the e-stop (green LED on VCU panel).
- Set the **physical remote-control switch on the VCU panel to AUTO**.

### Check 5: Are two arbitrators running?
```bash
ros2 node list | grep arbitrator
```
You should see exactly **one** node: `/command_arbitrator`.  
If you see a second node (e.g. `/control_arbitrator`), kill it immediately — it is the deleted
duplicate and will cause random STANDBY/AUTONOMOUS flapping.

---

## 7. Ending the Test

Press `Ctrl+C` in both terminals, then press the physical E-Stop button to put the vehicle back
into STANDBY.
