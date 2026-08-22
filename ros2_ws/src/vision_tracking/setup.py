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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tungt',
    maintainer_email='tungt@todo.todo',
    description='Phase 2: YOLOv8 + ByteTrack person tracking and PID gimbal control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = vision_tracking.yolo_detector_node:main',
            'gimbal_controller_node = vision_tracking.gimbal_controller_node:main',
            'tracking_eval = vision_tracking.tracking_eval:main',
        ],
    },
)
