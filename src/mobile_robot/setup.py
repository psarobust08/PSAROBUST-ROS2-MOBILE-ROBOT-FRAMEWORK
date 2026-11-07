from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fsm_node = mobile_robot.fsm_node:main',
            'robot_controller_node = mobile_robot.robot_controller_node:main',
            'camera_node = mobile_robot.camera_node:main',
            'detector_node = mobile_robot.detector_node:main',
            'status_node = mobile_robot.status_node:main',
            'web_node = mobile_robot.web_node:main',
            'serial_node = mobile_robot.serial_node:main',
        ],
    },

)
