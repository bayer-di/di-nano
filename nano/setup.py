#!/usr/bin/python
# -*- coding: utf-8 -*-

from setuptools import setup
from glob import glob
import os

package_name = 'nano'

setup(
    name=package_name,
    version='1.0.0',
    packages=[
        package_name,
        package_name + '.core',
        package_name + '.core.command',
        package_name + '.core.common',
        package_name + '.core.mqtt',
        package_name + '.core.schemas',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinkhello',
    maintainer_email='bayer-di@163.com',
    description='ROS2与MQTT转换Node',
    license='mmp',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "nano = nano.nano_run:main"
        ],
    },
)
