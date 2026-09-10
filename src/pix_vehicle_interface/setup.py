from setuptools import setup
import os
from glob import glob

package_name = 'pix_vehicle_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.dbc')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manish Gupta',
    maintainer_email='manishgupta9479@gmail.com',
    description='Direct CAN driver and DBC encoder/decoder interface for PIXKIT Shuttle',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
