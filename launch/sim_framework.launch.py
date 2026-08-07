"""
sim_framework.launch.py — Simulation Launch File
=================================================
Starts the PIXKIT control stack against the vehicle simulator.
All nodes except CAN RX/TX; includes vehicle_simulator and RViz2.

Usage:
  ros2 launch launch/sim_framework.launch.py
  ros2 launch launch/sim_framework.launch.py profile:=simulation
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='simulation',
        description='Configuration profile: simulation | hardware | tuning'
    )
    profile = LaunchConfiguration('profile')

    safety_cfg = os.path.join(
        get_package_share_directory('pix_safety_manager'), 'config', 'safety_params.yaml')
    arb_cfg    = os.path.join(
        get_package_share_directory('pix_command_manager'), 'config', 'arbitrator_params.yaml')
    sim_cfg    = os.path.join(
        get_package_share_directory('pix_simulator'), 'config', 'simulator_params.yaml')
    diag_cfg   = os.path.join(
        get_package_share_directory('pix_diagnostics'), 'config', 'diagnostics_params.yaml')
    log_cfg    = os.path.join(
        get_package_share_directory('pix_logger'), 'config', 'logger_params.yaml')
    state_cfg  = os.path.join(
        get_package_share_directory('pix_state_manager'), 'config', 'state_manager_params.yaml')

    return LaunchDescription([
        profile_arg,

        # 1. Command Arbitrator
        Node(
            package='pix_command_manager',
            executable='command_arbitrator',
            name='command_arbitrator',
            output='screen',
            parameters=[arb_cfg, {'active_timeout': 0.4}]
        ),

        # 2. Safety Manager (relaxed limits for simulation)
        Node(
            package='pix_safety_manager',
            executable='safety_manager',
            name='safety_manager',
            output='screen',
            parameters=[
                safety_cfg,
                {
                    'max_steer_angle':  350.0,
                    'max_steer_rate':   300.0,
                    'max_speed':          5.0,
                    'max_accel':          3.0,
                    'watchdog_timeout':   0.6,
                }
            ]
        ),

        # 3. Vehicle Simulator (replaces CAN RX/TX)
        Node(
            package='pix_simulator',
            executable='vehicle_simulator',
            name='vehicle_simulator',
            output='screen',
            parameters=[sim_cfg]
        ),

        # 4. System State Manager
        Node(
            package='pix_state_manager',
            executable='system_state_manager',
            name='pix_system_state_manager',
            output='screen',
            parameters=[state_cfg]
        ),

        # 5. Diagnostics
        Node(
            package='pix_diagnostics',
            executable='diagnostics_node',
            name='pix_diagnostics',
            output='screen',
            parameters=[diag_cfg]
        ),

        # 6. Logger
        Node(
            package='pix_logger',
            executable='logger_node',
            name='pix_logger',
            output='screen',
            parameters=[log_cfg]
        ),

        # 7. Config Manager
        Node(
            package='pix_config_manager',
            executable='config_manager',
            name='pix_config_manager',
            output='screen',
            parameters=[{'profile': profile}]
        ),

        # 8. YOLO Person Avoidance (with display for sim debugging)
        Node(
            package='yolo_person_avoidance',
            executable='yolo_avoidance',
            name='yolo_avoidance_node',
            output='screen',
            parameters=[{
                'yolo_model':           'yolov8n.pt',
                'confidence_threshold':  0.40,
                'gain':                300.0,
                'max_avoidance':       500.0,
                'deadband':              0.08,
                'ramp_rate':           200.0,
                'hold_frames':           15,
                'speed_dps':           250.0,
                'target_speed':          2.0,
                'no_display':           False,  # show window in sim
            }]
        ),

        # 9. RViz2 (sim visualization only)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
