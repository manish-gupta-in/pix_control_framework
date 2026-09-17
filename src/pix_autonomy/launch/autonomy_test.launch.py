import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('pix_autonomy'), 'config')
    params_file = os.path.join(config_dir, 'autonomy_params.yaml')

    return LaunchDescription([
        # 2. The Eyes
        Node(
            package='pix_autonomy',
            executable='yolo_perception_node',
            name='yolo_perception',
            parameters=[params_file],
            output='screen'
        ),
        # 3. The Reflexes
        Node(
            package='pix_autonomy',
            executable='aeb_node',
            name='aeb_node',
            parameters=[params_file],
            output='screen'
        ),
        # 4. The Driver (Straight line test)
        Node(
            package='pix_autonomy',
            executable='straight_drive_node',
            name='straight_drive_planner',
            parameters=[params_file],
            output='screen'
        )
    ])
