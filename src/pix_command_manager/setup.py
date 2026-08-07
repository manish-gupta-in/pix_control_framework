import os
from setuptools import setup
from glob import glob

package_name = 'pix_command_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bits',
    maintainer_email='bits@todo.todo',
    description='Command router and priority-based command arbitration node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_arbitrator = pix_command_manager.command_arbitrator:main',
        ],
    },
)
