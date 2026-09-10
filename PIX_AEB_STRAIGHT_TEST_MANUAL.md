# PIX AEB & Straight Line Test Manual (v15 Fixed)

This manual provides step-by-step instructions for performing a **50-meter straight line test** with the **Autonomous Emergency Braking (AEB)** system active. 

In this test, the vehicle will drive straight forward at a constant speed, and if a person steps into the camera view, the vehicle will slam the brakes to stop immediately.

---

## 1. Safety Prerequisites
- ⚠ Ensure you have at least **50 meters of completely clear, flat ground** ahead of the vehicle.
- ⚠ Ensure the **Hardware E-Stop button** (on the vehicle or RC controller) is within reach at all times.
- Ensure the VCU physical switch is set to **AUTO**.

---

## 2. Setting up the Vehicle
1. Unzip `pix_control_framework_v15_fixed.zip` on the vehicle computer.
2. Build the workspace (only needs to be done once):
   ```bash
   cd pix_control_framework
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```
3. Ensure the CAN interface is up:
   ```bash
   sudo ip link set can4 up type can bitrate 500000
   sudo ip link set can4 txqueuelen 1000
   ```

---

## 3. Running the Test (Step-by-Step)

You will need to open **4 separate terminals**. In every terminal, ensure you source the workspace first:
```bash
cd ~/pix_control_framework
source install/setup.bash
```

### Terminal 1: Core Framework (Hardware Interface)
This brings up the CAN driver and Safety Manager.
```bash
ros2 launch launch/hw_framework.launch.py
```
*(Verify that the CAN connection is established and the VCU reports "Auto" mode).*

### Terminal 2: Control Arbitrator
This node acts as the traffic cop, listening to your driving and braking commands.
```bash
ros2 run pix_autonomy control_arbitrator_node
```

### Terminal 3: Launch the Entire Autonomy Stack
Instead of running 4 separate commands, you can now launch the entire autonomy test (Arbitrator, YOLO, AEB, and Straight Driver) with one single command!
```bash
ros2 launch pix_autonomy autonomy_test.launch.py
```

⚠ **WARNING: As soon as you run this command, the vehicle will start moving forward.**

---

## 4. How to Tune Parameters (Speed, AEB distance, etc.)
You do **NOT** need to rebuild the workspace to change speeds or AEB settings.

Simply open the parameter configuration file on the vehicle:
`src/pix_autonomy/config/autonomy_params.yaml`

In this file, you can edit:
- **`speed: 1.5`** (Under `straight_drive_planner`) - Change the testing speed.
- **`ttc_threshold: 2.0`** (Under `aeb_node`) - Increase this to make it brake earlier (e.g. 3.0), or decrease it to brake later.

After saving the `.yaml` file, just rerun the `ros2 launch` command in Terminal 3!

## 5. The Test Procedure
1. The vehicle will now drive perfectly straight at `1.0 m/s`.
2. Have a person safely walk into the camera frame from the side (at least 5-10 meters ahead).
3. **EXPECTED BEHAVIOR**:
   - `yolo_perception_node` will detect the person and publish the distance.
   - `aeb_node` will calculate the Time-To-Collision (TTC). If TTC is under 2.0 seconds, it will override the `straight_drive_node`.
   - The vehicle will immediately brake at 100% capacity.
   - The terminal running `aeb_node` will print: `[ERROR] AEB ENGAGED! TTC: x.xxs, Dist: y.yym`

### Testing Different Speeds
Once comfortable, hit `Ctrl+C` in Terminal 4 to stop driving. Then restart it with a higher speed:

```bash
# Test at 2.5 m/s (~9 km/h)
ros2 run pix_autonomy straight_drive_node --ros-args -p speed:=2.5
```

## 5. Ending the Test
When you are finished, press `Ctrl+C` in all terminals and press the physical E-Stop button on the vehicle to place it back into STANDBY mode.
