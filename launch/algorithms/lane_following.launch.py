"""
lane_following.launch.py — Lane Following Algorithm
====================================================
Launch this IN A SEPARATE TERMINAL after the core interface is running.

  Terminal 2 (this algorithm):
    ros2 launch launch/algorithms/lane_following.launch.py

Algorithm publishes to:  /pix/commands/lane_following
Priority in arbitrator:  5 (lowest — overridden by avoidance)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_following',
            executable='lane_following_node',
            name='lane_following_node',
            output='screen',
            parameters=[{
                'gain_lateral':  0.005,
                'target_speed':  1.5,
                'lookahead_m':   2.5,
            }]
        ),
    ])
