"""
yolo_avoidance.launch.py — YOLO Person Avoidance Algorithm
===========================================================
Launch this IN A SEPARATE TERMINAL after the core interface is running:

  Terminal 1 (core interface):
    ros2 launch launch/hw_framework.launch.py profile:=hardware

  Terminal 2 (this algorithm):
    ros2 launch launch/algorithms/yolo_avoidance.launch.py

  Stop algorithm:  Ctrl+C in Terminal 2  (core interface keeps running)

Algorithm publishes to:  /pix/commands/human_avoidance
Priority in arbitrator:  3 (above lane following, below collision avoidance)

STEER_ONLY_MODE:
  True  = Only steer, vehicle does NOT move. Safe for stationary tests.
  False = Full avoidance with speed control (set target_speed).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    steer_only_arg = DeclareLaunchArgument(
        'steer_only', default_value='true',
        description='true = steer only (safe stationary test), false = full avoidance'
    )
    steer_only = LaunchConfiguration('steer_only')

    return LaunchDescription([
        steer_only_arg,

        Node(
            package='yolo_person_avoidance',
            executable='yolo_avoidance',
            name='yolo_avoidance_node',
            output='screen',
            parameters=[{
                'yolo_model':           'yolov8n.pt',
                'confidence_threshold':  0.40,
                'gain':                300.0,
                'max_avoidance':       200.0,   # max steer angle for avoidance
                'deadband':              0.08,
                'ramp_rate':           100.0,
                'hold_frames':           15,
                'speed_dps':           100.0,
                'target_speed':          1.5,   # m/s when steer_only=False
                'no_display':           False,  # show camera window
                'steer_only_mode':       True,  # safe default: steer only
            }]
        ),
    ])
