from setuptools import setup
import os, glob

package_name = 'pix_diagnostics'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob.glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PIX Control Framework',
    maintainer_email='pix@example.com',
    description='Diagnostics framework for PIXKIT',
    license='MIT',
    entry_points={
        'console_scripts': [
            'diagnostics_node = pix_diagnostics.diagnostics_node:main',
        ],
    },
)
