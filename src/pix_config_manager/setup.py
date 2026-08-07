from setuptools import setup
import os, glob
package_name = 'pix_config_manager'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'profiles'), glob.glob(os.path.join('profiles', '*.yaml'))),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='PIX Control Framework', maintainer_email='pix@example.com',
    description='Configuration profile manager for PIXKIT', license='MIT',
    entry_points={'console_scripts': ['config_manager = pix_config_manager.config_manager_node:main']},
)
