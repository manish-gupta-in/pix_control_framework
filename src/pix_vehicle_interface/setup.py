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
    ],
    install_requires=['setuptools', 'cantools', 'python-can'],
    zip_safe=True,
    maintainer='bits',
    maintainer_email='bits@todo.todo',
    description='Direct CAN driver and DBC encoder/decoder interface for PIXKIT Shuttle',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_tx = pix_vehicle_interface.can_tx:main',
            'can_rx = pix_vehicle_interface.can_rx:main',
        ],
    },
)
