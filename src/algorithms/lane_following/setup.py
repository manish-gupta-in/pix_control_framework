from setuptools import setup

package_name = 'lane_following'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bits',
    maintainer_email='bits@todo.todo',
    description='Lane following algorithm interface mock',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_following = lane_following.lane_following_node:main',
        ],
    },
)
