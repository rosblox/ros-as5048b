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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosblox',
    maintainer_email='info@rosblox.com',
    description='ROS AS5048b package.',
    license='BSD3',
    entry_points={
        'console_scripts': [
                'ros_as5048b_publisher = ros_as5048b.publisher_member_function:main',
        ],
    },
)
