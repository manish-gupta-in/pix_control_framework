from setuptools import setup

package_name = 'pix_algorithm_api'

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
    maintainer='Manish Gupta',
    maintainer_email='manishgupta9479@gmail.com',
    description='Base interface and API for modular autonomous driving algorithms',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
