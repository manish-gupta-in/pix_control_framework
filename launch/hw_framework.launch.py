"""
hw_framework.launch.py — Hardware Deployment Launch File
=========================================================
Starts the full PIXKIT control stack for real vehicle deployment.
All 9 nodes: CAN RX/TX, Command Arbitrator, Safety Manager, State Manager,
Diagnostics, Logger, Config Manager, YOLO Person Avoidance.

Usage:
  ros2 launch launch/hw_framework.launch.py
  ros2 launch launch/hw_framework.launch.py profile:=hardware

Pre-launch checklist:
  1. sudo ip link set can4 up type can bitrate 500000
  2. candump can4 -n 10  (confirm VCU frames on 0x500-0x512)
  3. ros2 topic echo /pix/vehicle_status  (confirm RX decode)
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
    safety_cfg  = os.path.join(
        get_package_share_directory('pix_safety_manager'), 'config', 'safety_params.yaml')
    arb_cfg     = os.path.join(
        get_package_share_directory('pix_command_manager'), 'config', 'arbitrator_params.yaml')
    can_rx_cfg  = os.path.join(
        get_package_share_directory('pix_vehicle_interface'), 'config', 'can_rx_params.yaml')
    can_tx_cfg  = os.path.join(
        get_package_share_directory('pix_vehicle_interface'), 'config', 'can_tx_params.yaml')
    diag_cfg    = os.path.join(
        get_package_share_directory('pix_diagnostics'), 'config', 'diagnostics_params.yaml')
    log_cfg     = os.path.join(
        get_package_share_directory('pix_logger'), 'config', 'logger_params.yaml')
    state_cfg   = os.path.join(
        get_package_share_directory('pix_state_manager'), 'config', 'state_manager_params.yaml')

    return LaunchDescription([
        profile_arg,

        # 1. CAN RX — reads and decodes VCU frames → /pix/vehicle_status
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

        # 2. CAN TX — encodes /pix/control_cmd → raw CAN frames
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

        # 3. Command Arbitrator — priority mux of algorithm sources
        Node(
            package='pix_command_manager',
            executable='command_arbitrator',
            name='command_arbitrator',
            output='screen',
            parameters=[arb_cfg, {'active_timeout': 0.4}]
        ),

        # 4. Safety Manager — rate limits, bounds, watchdog
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

        # 5. System State Manager — MANUAL/STANDBY/AUTONOMOUS/FAULT/ESTOP
        Node(
            package='pix_state_manager',
            executable='system_state_manager',
            name='pix_system_state_manager',
            output='screen',
            parameters=[state_cfg]
        ),

        # 6. Diagnostics — /diagnostics publisher (CAN, VCU, battery, state)
        Node(
            package='pix_diagnostics',
            executable='diagnostics_node',
            name='pix_diagnostics',
            output='screen',
            parameters=[diag_cfg]
        ),

        # 7. Logger — CSV logs to ~/pix_logs/<session>/
        Node(
            package='pix_logger',
            executable='logger_node',
            name='pix_logger',
            output='screen',
            parameters=[log_cfg]
        ),

        # 8. Config Manager — loads and broadcasts active profile
        Node(
            package='pix_config_manager',
            executable='config_manager',
            name='pix_config_manager',
            output='screen',
            parameters=[{'profile': profile}]
        ),

        # 9. YOLO Person Avoidance (headless for HW deployment)
        Node(
            package='yolo_person_avoidance',
            executable='yolo_avoidance',
            name='yolo_avoidance_node',
            output='screen',
            parameters=[{
                'yolo_model':            'yolov8n.pt',
                'confidence_threshold':   0.40,
                'gain':                 300.0,
                'max_avoidance':        500.0,
                'deadband':               0.08,
                'ramp_rate':            200.0,
                'hold_frames':            15,
                'speed_dps':            250.0,
                'target_speed':           2.0,
                'no_display':            True,   # headless for vehicle
            }]
        ),
    ])
