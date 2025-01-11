from setuptools import setup
import os
from glob import glob

package_name = 'param_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Include all launch files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shashank Goyal',
    maintainer_email='shashank3199@gmail.com',
    description='ROS2 parameter demonstration package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'py_read_node = param_demo.py_read_node:main',
            'py_write_node = param_demo.py_write_node:main',
            'py_modify_node = param_demo.py_modify_node:main',
        ],
    },
)