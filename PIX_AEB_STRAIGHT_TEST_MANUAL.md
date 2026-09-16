# PIX AEB & Straight-Line Test Manual — v15.0

> Step-by-step procedure for performing a **50-metre straight-line test** with the **Autonomous Emergency Braking (AEB)** system active on a real PIX vehicle.

**Test goal:** Vehicle drives forward at constant speed. If a person enters the camera frame, the AEB system fires and the vehicle stops immediately.

---

## Safety Prerequisites

> [!CAUTION]
> This test involves a moving vehicle. Do NOT proceed unless all conditions below are met.

- ⚠️ **Clear space:** Minimum 50 metres of flat, unobstructed ground ahead
- ⚠️ **E-Stop ready:** Hardware E-stop (vehicle or RC controller) within operator reach at all times
- ⚠️ **VCU switch:** Physical VCU switch set to **AUTO** mode
- ⚠️ **Personnel:** Only designated test personnel within the test zone

---

## 1. Vehicle Setup

### 1.1 Build the workspace *(first time only)*

```bash
cd ~/pix_control_framework
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 1.2 Bring up the CAN interface

```bash
sudo ip link set can4 up type can bitrate 500000
sudo ip link set can4 txqueuelen 1000
```

Verify: `ip link show can4` should show `UP` state.

---

## 2. Running the Test

Open **two terminals** on the vehicle computer. In each terminal:

```bash
cd ~/pix_control_framework
source install/setup.bash
```

### Terminal 1 — Core Hardware Stack

```bash
ros2 launch launch/hw_framework.launch.py
```

**Expected:** CAN connection established, VCU reports `AUTO` mode. Wait for confirmation before proceeding.

### Terminal 2 — Full Autonomy Stack

```bash
ros2 launch pix_autonomy autonomy_test.launch.py
```

> ⚠️ **The vehicle starts moving as soon as this command runs.**

This single launch file starts: Straight Drive Node + YOLO Perception + AEB Node + Control Arbitrator.

---

## 3. Test Procedure

1. Verify Terminal 1 shows CAN is up and VCU is in AUTO mode.
2. Run Terminal 2. The vehicle will drive straight at **1.0 m/s** (configurable).
3. Have a test person safely **walk into the camera frame** from the side — at least 5–10 m ahead.

### Expected Behaviour

| Step | What happens |
|---|---|
| Person detected | `yolo_perception_node` publishes obstacle distance to `/perception/obstacles` |
| TTC calculated | `aeb_node` computes Time-To-Collision |
| TTC < 2.0s | AEB fires — overrides straight drive command |
| Vehicle response | 100% braking applied — vehicle stops |
| Log output | `[ERROR] AEB ENGAGED! TTC: x.xxs, Dist: y.yym` |

---

## 4. Tuning Parameters

> No rebuild required. Edit the YAML file and relaunch Terminal 2.

**File:** `src/pix_autonomy/config/autonomy_params.yaml`

```yaml
straight_drive_planner:
  speed: 1.5          # m/s — change test speed here

aeb_node:
  ttc_threshold: 2.0  # seconds
                      # Higher (e.g. 3.0) = brakes earlier / more cautious
                      # Lower  (e.g. 1.5) = brakes later  / less cautious
```

### Speed Progression (recommended test sequence)

```bash
# Test at 1.5 m/s (~5.4 km/h) — default
ros2 launch pix_autonomy autonomy_test.launch.py

# Test at 2.5 m/s (~9 km/h) — advanced
ros2 run pix_autonomy straight_drive_node --ros-args -p speed:=2.5

# Test at 3.5 m/s (~12.6 km/h) — high speed
ros2 run pix_autonomy straight_drive_node --ros-args -p speed:=3.5
```

Always start at lower speeds and validate AEB before increasing.

---

## 5. Ending the Test

1. Press `Ctrl+C` in **Terminal 2** (autonomy stack stops — vehicle decelerates)
2. Press `Ctrl+C` in **Terminal 1** (hardware stack stops)
3. Press the **physical E-Stop** button to place the vehicle into STANDBY mode
4. Set the VCU physical switch back to **MANUAL**

---

## 6. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| Vehicle does not move | VCU not in AUTO mode | Check physical VCU switch |
| CAN errors in Terminal 1 | CAN interface not up | Re-run `ip link set can4 up` |
| AEB does not fire | YOLO not detecting person | Check camera topic, verify `yolov8n.pt` is present |
| Vehicle moves after Ctrl+C | Safety Manager still commanding | Press physical E-Stop |

---

*Part of [PIX Control Framework v15.0](README.md)*
