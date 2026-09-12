import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tungt',
    maintainer_email='tungt@todo.todo',
    description='YOLOv8 tracking and repeatable simulation realism/fault injection',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = vision_tracking.yolo_detector_node:main',
            'sim_realism_node = vision_tracking.sim_realism_node:main',
            'motion_arbiter_node = vision_tracking.motion_arbiter_node:main',
            'live_camera_hud_node = vision_tracking.live_camera_hud_node:main',
        ],
    },
)
