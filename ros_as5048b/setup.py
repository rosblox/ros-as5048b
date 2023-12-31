import os
from glob import glob
from setuptools import setup

package_name = 'ros_as5048b'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosblox',
    maintainer_email='info@rosblox.com',
    description='ROS AS5048b package.',
    license='BSD3',
    entry_points={
        'console_scripts': [
                'read_encoder_node = ros_as5048b.read_encoder_node:main',
                'turn_counter_node = ros_as5048b.turn_counter_node:main',
        ],
    },
)
