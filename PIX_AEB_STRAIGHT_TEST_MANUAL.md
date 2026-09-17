# PIX AEB & Straight-Line Test Manual

> **Version:** v16.0 | **Vehicle:** PIXKIT DTV | **Framework:** PIX Control Framework v16  
> **Procedure:** 50-metre straight-line drive with Autonomous Emergency Braking active

---

## ⚠ Safety Statement

This procedure involves a live vehicle moving under autonomous control. **Do not proceed unless all safety prerequisites below are satisfied.** The test operator retains full responsibility for safe execution. The hardware E-Stop must be reachable and operational at all times.

---

## Table of Contents

1. [Safety Prerequisites](#1-safety-prerequisites)
2. [Hardware Setup](#2-hardware-setup)
3. [Software Setup](#3-software-setup)
4. [Test Procedure](#4-test-procedure)
5. [Parameter Reference](#5-parameter-reference)
6. [Expected Behaviour Checklist](#6-expected-behaviour-checklist)
7. [How to Stop Safely](#7-how-to-stop-safely)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Safety Prerequisites

Verify every item before powering the vehicle:

| # | Check | Requirement |
|:---:|---|---|
| 1 | **Test area** | Minimum 50 m of flat, clear, obstacle-free ground ahead of the vehicle. Side clearance ≥ 3 m. |
| 2 | **E-Stop button** | Hardware E-Stop (on vehicle or RC controller) is within reach of the operator at all times. |
| 3 | **VCU mode** | Physical VCU remote-control selector is set to **AUTO**. |
| 4 | **CAN interface** | `can4` is up at 500 kbps (verified with `candump can4`). |
| 5 | **Workspace built** | `colcon build` completed without errors; `install/setup.bash` is sourced. |
| 6 | **Camera** | USB / GigE camera is connected and publishing on `/camera/right/image`. |
| 7 | **YOLO weights** | `yolov8n.pt` is present in the workspace root. |
| 8 | **Test personnel** | A second person is present to trigger E-Stop if needed. Never perform this test alone. |

> ⚠ **Do not skip any item above.** A missed check is a safety incident waiting to happen.

---

## 2. Hardware Setup

### 2.1 CAN Interface

Run once after each boot:

```bash
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000
```

Verify:

```bash
candump can4
# You should see periodic CAN frames from the VCU
```

### 2.2 VCU Mode Verification

```bash
# Confirm VCU is in AUTO mode: ModeState byte = 0x01
candump can4 | grep 505
# Expected output example: can4  505   [8]  00 20 01 00 00 00 00 00
#                                           ^^^^ ModeState = 1 (AUTO)
```

If ModeState shows `03` the VCU is in STANDBY — flip the physical selector to AUTO.

---

## 3. Software Setup

### 3.1 Clone & Build (first time only)

```bash
git clone git@github.com:manish-gupta-in/pix_control_framework.git
cd pix_control_framework
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3.2 Source the Workspace

Run this in **every new terminal** before any `ros2` command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 4. Test Procedure

The test requires **two terminals**. Source the workspace in each before proceeding.

---

### Terminal 1 — Core Hardware Framework

Brings up the CAN driver, Safety Manager, State Manager, and Diagnostics. **Must be running before Terminal 2.**

```bash
ros2 launch launch/hw_framework.launch.py
```

**✅ Success indicators:**
- `[pix_can_driver]` logs: `CAN interface can4 opened`
- `[pix_safety_manager]` logs: `Safety Manager initialised`
- `[system_state_manager]` logs: `State: STANDBY`
- No `ERROR` or `FATAL` messages in the first 10 seconds

---

### Terminal 2 — Autonomy Stack

Launches the complete autonomy test in a single command:
- `yolo_perception_node` — person detection
- `aeb_node` — emergency braking
- `straight_drive_node` — constant forward drive
- `control_arbitrator_node` — command priority MUX

```bash
ros2 launch src/pix_autonomy/launch/autonomy_test.launch.py
```

> ⚠ **WARNING: The vehicle will begin moving forward as soon as this launch completes.**  
> Ensure the test area is clear and your hand is near the E-Stop before running this command.

**✅ Success indicators:**
- `[yolo_perception_node]` logs: `YOLO model loaded from yolov8n.pt`
- `[straight_drive_node]` logs: `Publishing drive command at X.X m/s`
- `[control_arbitrator_node]` logs: `Arbitrator running at 50 Hz`
- Vehicle begins moving forward

---

## 5. Parameter Reference

All parameters are in `src/pix_autonomy/config/autonomy_params.yaml`. **No rebuild required** — edit and re-run the launch command.

| Parameter | Node | Default | Description |
|---|---|---|---|
| `speed` | `straight_drive_node` | `1.5` m/s | Forward test speed |
| `ttc_threshold` | `aeb_node` | `2.0` s | Brake when TTC falls below this value. Increase to brake earlier. |
| `model_path` | `yolo_perception_node` | `yolov8n.pt` | Path to YOLO weights file |
| `camera_topic` | `yolo_perception_node` | `/camera/right/image` | Camera topic to subscribe to |

### Runtime Override (no file edit)

```bash
# Test at 2.5 m/s with tighter AEB (1.5 s TTC)
ros2 launch src/pix_autonomy/launch/autonomy_test.launch.py \
    straight_drive_node:speed:=2.5 \
    aeb_node:ttc_threshold:=1.5
```

Or run nodes individually:

```bash
ros2 run pix_autonomy straight_drive_node --ros-args -p speed:=2.5
ros2 run pix_autonomy aeb_node --ros-args -p ttc_threshold:=3.0
```

---

## 6. Expected Behaviour Checklist

### Normal Operation (no obstacle)

| Step | What Happens |
|---|---|
| 1 | Vehicle drives straight at the configured speed (default 1.5 m/s) |
| 2 | `straight_drive_node` publishes to `/pix_autonomy/straight_cmd` |
| 3 | `control_arbitrator_node` selects `straight` (lowest active priority) |
| 4 | `/pix/control_cmd` flows to `pix_vehicle_interface_cpp` at 50 Hz |

### AEB Trigger (obstacle in camera view)

| Step | What Happens |
|---|---|
| 1 | Person walks into the camera frame 5–10 m ahead |
| 2 | `yolo_perception_node` detects `person` class, publishes distance to `/perception/obstacles` |
| 3 | `aeb_node` computes: `TTC = distance / speed` |
| 4 | If `TTC < 2.0 s`: AEB publishes to `/pix_autonomy/aeb_cmd` with `brake_target=100`, `emergency_stop=True` |
| 5 | `control_arbitrator_node` immediately selects `aeb` (highest priority) |
| 6 | Vehicle brakes at full capacity and stops |
| 7 | Terminal shows: `[ERROR] AEB ENGAGED! TTC: X.XXs, Dist: Y.YYm` |

### Recovery After AEB

Once the obstacle clears:
1. `aeb_node` stops publishing (no detection → no command)
2. Arbitrator falls back to `straight_drive_node`
3. Vehicle resumes forward motion

---

## 7. How to Stop Safely

### Graceful Stop (software)

Press **Ctrl + C** in Terminal 2 first, then Terminal 1. The Safety Manager's watchdog will detect the command timeout (300 ms) and command a stop automatically.

### Emergency Stop (hardware)

Press the **physical E-Stop button** on the vehicle or RC controller at any time. This immediately disengages the drive and overrides all software commands. The VCU returns to STANDBY mode.

After pressing E-Stop:
1. Press Ctrl + C in both terminals
2. Inspect the vehicle and test area before re-engaging

---

## 8. Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| Vehicle does not move | VCU in STANDBY | Set physical VCU selector to **AUTO**; verify `candump` shows ModeState=1 |
| `YOLO model loaded` not seen | `yolov8n.pt` not found | Confirm file is in workspace root: `ls yolov8n.pt` |
| `Missing dependencies` error | `ultralytics` / `cv_bridge` not installed | `pip install ultralytics && sudo apt install ros-humble-cv-bridge` |
| No camera frames | Wrong camera topic | Set `camera_topic` parameter to your actual camera topic |
| AEB not triggering | Person too far away | Walk closer (within 3 m for 1.5 m/s with 2 s TTC threshold) |
| AEB triggering too early | `ttc_threshold` too high | Reduce `ttc_threshold` in `autonomy_params.yaml` |
| CAN errors in Terminal 1 | `can4` interface not up | Run: `sudo ip link set can4 up type can bitrate 500000` |
| `colcon build` fails | Missing ROS 2 packages | `rosdep install --from-paths src --ignore-src -r -y` |
| Watchdog E-stop fires | Command loop stalled | Check node is running: `ros2 node list | grep autonomy` |
