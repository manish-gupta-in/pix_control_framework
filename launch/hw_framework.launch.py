"""
hw_framework.launch.py — PIXKIT CAN Interface Core (Algorithm-Free)
====================================================================
Starts ONLY the core CAN interface nodes. NO algorithm nodes are included.
Algorithms are launched separately via their own launch files.

This mirrors the Whale/Autoware modular architecture:
  ┌─────────────────────────────────────────────────────────┐
  │  CORE INTERFACE (this file)                             │
  │   can_rx  →  /pix/vehicle_status                        │
  │   can_tx  ←  /pix/control_cmd                          │
  │   command_arbitrator  (priority MUX)                    │
  │   safety_manager      (rate-limit, bounds, watchdog)    │
  │   system_state_manager (STANDBY/AUTONOMOUS/FAULT)       │
  │   diagnostics_node    (/diagnostics)                    │
  │   logger_node         (CSV logs)                        │
  │   config_manager      (profile loader)                  │
  └─────────────────────────────────────────────────────────┘
       ↑ Algorithms subscribe to /pix/vehicle_status and
         publish to /pix/commands/<algorithm_name>

Algorithm launch files (run in a SEPARATE terminal AFTER this):
  ros2 launch launch/algorithms/yolo_avoidance.launch.py
  ros2 launch launch/algorithms/lane_following.launch.py
  ros2 run <pkg> <node>  (any custom algorithm)

VCU Gear Change Pre-requisite (HARDWARE INTERLOCK):
  ⚠ IMPORTANT: The physical remote-control switch on the VCU panel
    MUST be in AUTO mode for gear commands to be accepted.
    In STANDBY mode (Vehicle_ModeState=3), the VCU ignores Gear_EnCtrl.
    Steps:
      1. Ensure e-stop is disengaged (green LED on VCU)
      2. Set the VCU remote selector to AUTO position
      3. Confirm: candump can4 | grep 505 shows ...20 01 (ModeState=1)
      4. Only then will gear changes work via ROS commands

Usage:
  ros2 launch launch/hw_framework.launch.py
  ros2 launch launch/hw_framework.launch.py profile:=hardware

CAN Pre-launch checklist:
  1. sudo ip link set can4 up type can bitrate 500000
  2. sudo ip link set can4 txqueuelen 1000
  3. candump can4 -n 5  (confirm VCU frames on 0x500–0x512)
  4. Check Vehicle_ModeState byte in 0x505 frame byte[4] bits[3:2]
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='hardware',
        description='Configuration profile: simulation | hardware | tuning'
    )
    profile = LaunchConfiguration('profile')

    # ── Config file paths ─────────────────────────────────────────────────────
    safety_cfg = os.path.join(
        get_package_share_directory('pix_safety_manager'), 'config', 'safety_params.yaml')
    arb_cfg    = os.path.join(
        get_package_share_directory('pix_command_manager'), 'config', 'arbitrator_params.yaml')
    can_rx_cfg = os.path.join(
        get_package_share_directory('pix_vehicle_interface'), 'config', 'can_rx_params.yaml')
    can_tx_cfg = os.path.join(
        get_package_share_directory('pix_vehicle_interface'), 'config', 'can_tx_params.yaml')
    diag_cfg   = os.path.join(
        get_package_share_directory('pix_diagnostics'), 'config', 'diagnostics_params.yaml')
    log_cfg    = os.path.join(
        get_package_share_directory('pix_logger'), 'config', 'logger_params.yaml')
    state_cfg  = os.path.join(
        get_package_share_directory('pix_state_manager'), 'config', 'state_manager_params.yaml')

    return LaunchDescription([
        profile_arg,

        # ── 1. CAN RX ─────────────────────────────────────────────────────────
        # Decodes all VCU CAN frames → /pix/vehicle_status at 50 Hz
        # Frames: 0x500 Throttle, 0x501 Brake, 0x502 Steer, 0x503 Gear,
        #         0x504 Park, 0x505 VCU_Report, 0x506 WheelSpeed, 0x512 BMS
        Node(
            package='pix_vehicle_interface',
            executable='can_rx',
            name='can_rx',
            output='screen',
            parameters=[
                can_rx_cfg,
                {'can_interface': 'can4', 'loop_rate': 50.0, 'enable_can_rx': True},
            ]
        ),

        # ── 2. CAN TX ─────────────────────────────────────────────────────────
        # Encodes /pix/control_cmd → 6 CAN TX frames at 50 Hz:
        #   0x100 Throttle, 0x101 Brake, 0x102 Steer,
        #   0x103 Gear, 0x104 Park, 0x105 Vehicle_Mode_Command
        # Auto_Professional=1 is ALWAYS held HIGH to signal autonomous intent.
        # NOTE: VCU still requires physical key/remote in AUTO position to
        # actually enter Auto Mode and accept gear commands.
        Node(
            package='pix_vehicle_interface',
            executable='can_tx',
            name='can_tx',
            output='screen',
            parameters=[
                can_tx_cfg,
                {'can_interface': 'can4', 'loop_rate': 50.0, 'enable_can_tx': True},
            ]
        ),

        # ── 3. Command Arbitrator ─────────────────────────────────────────────
        # Priority multiplexer for algorithm command topics:
        #   Priority 1 (highest): /pix/commands/emergency_stop
        #   Priority 2: /pix/commands/collision_avoidance
        #   Priority 3: /pix/commands/human_avoidance
        #   Priority 4: /pix/commands/cruise_control
        #   Priority 5: /pix/commands/lane_following (lowest)
        # Output: /pix/raw_control_cmd (→ safety_manager → can_tx)
        # Any new algorithm just needs to publish to its registered topic.
        Node(
            package='pix_command_manager',
            executable='command_arbitrator',
            name='command_arbitrator',
            output='screen',
            parameters=[arb_cfg, {'active_timeout': 0.4}]
        ),

        # ── 4. Safety Manager ─────────────────────────────────────────────────
        # Enforces hard limits before forwarding to CAN TX:
        #   - Max steer angle: 280°
        #   - Max speed: 3.0 m/s (hardware profile)
        #   - Max acceleration: 1.0 m/s²
        #   - Watchdog timeout: 0.3 s → sends zero-speed safe cmd
        # Input:  /pix/raw_control_cmd
        # Output: /pix/control_cmd
        Node(
            package='pix_safety_manager',
            executable='safety_manager',
            name='safety_manager',
            output='screen',
            parameters=[
                safety_cfg,
                {
                    'max_steer_angle':   280.0,
                    'max_steer_rate':    150.0,
                    'max_speed':           3.0,
                    'max_accel':           1.0,
                    'watchdog_timeout':    0.3,
                }
            ]
        ),

        # ── 5. System State Manager ───────────────────────────────────────────
        # Tracks vehicle state: MANUAL → STANDBY → AUTONOMOUS → FAULT/ESTOP
        # Publishes /pix/system_state
        # STANDBY = no algorithm publishing
        # AUTONOMOUS = algorithm actively commanding vehicle
        Node(
            package='pix_state_manager',
            executable='system_state_manager',
            name='pix_system_state_manager',
            output='screen',
            parameters=[state_cfg]
        ),

        # ── 6. Diagnostics ────────────────────────────────────────────────────
        # Publishes /diagnostics with health checks for:
        #   CAN RX age, CAN TX age, VCU faults, Battery (BMS), State
        Node(
            package='pix_diagnostics',
            executable='diagnostics_node',
            name='pix_diagnostics',
            output='screen',
            parameters=[diag_cfg]
        ),

        # ── 7. Logger ─────────────────────────────────────────────────────────
        # CSV logs to ~/pix_logs/<session>/  at 10 Hz
        Node(
            package='pix_logger',
            executable='logger_node',
            name='pix_logger',
            output='screen',
            parameters=[log_cfg]
        ),

        # ── 8. Config Manager ─────────────────────────────────────────────────
        # Loads and broadcasts profile YAML → /pix/config/active_profile
        Node(
            package='pix_config_manager',
            executable='config_manager',
            name='pix_config_manager',
            output='screen',
            parameters=[{'profile': profile}]
        ),

        # ── NO ALGORITHM NODES HERE ───────────────────────────────────────────
        # Launch algorithms separately in another terminal:
        #   ros2 launch launch/algorithms/yolo_avoidance.launch.py
        #   ros2 launch launch/algorithms/lane_following.launch.py
        # This way the core CAN interface stays alive even if an algorithm
        # crashes, and you can hot-swap algorithms without restarting.

    ])
