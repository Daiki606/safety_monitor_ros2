# SPDX-License-Identifier: MIT

from setuptools import setup

package_name = 'ros2_safety_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daiki Yamashita',
    maintainer_email='s24c1131sc@s.chibakoudai.jp',
    description='ROS2 node that monitors robot safety zones.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = ros2_safety_monitor.safety_node:main',
        ],
    },
)
