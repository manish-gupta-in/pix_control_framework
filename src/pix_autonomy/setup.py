from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pix_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manish Gupta',
    maintainer_email='manishgupta9479@gmail.com',
    description='Autonomy stack examples for PIX Control Framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_waypoint_follower = pix_autonomy.gnss_waypoint_follower:main',
            'aeb_node = pix_autonomy.aeb_node:main',
            'mpc_planner_node = pix_autonomy.mpc_planner_node:main',
            'yolo_perception_node = pix_autonomy.yolo_perception_node:main',
            'lateral_avoidance_node = pix_autonomy.lateral_avoidance_node:main',
            'straight_drive_node = pix_autonomy.straight_drive_node:main',
        ],
    },
)
