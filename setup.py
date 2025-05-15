from setuptools import setup
import os
from glob import glob

package_name = 'zmoab_uros_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='A helper package for ZMOAB with Micro-ROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_odom_converter = zmoab_uros_utils.vel_odom_converter:main',
            'laserscan_relay = zmoab_uros_utils.laserscan_relay:main'
        ],
    },
)
