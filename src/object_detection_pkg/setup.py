from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Automatically find Python packages
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # Correcting launch file pattern
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for object detection using YOLO and RealSense',
    license='Apache License 2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detection_node = object_detection_pkg.yolo_detection_node:main',
            'obstacle_processing_node = object_detection_pkg.obstacle_processing_node:main',
            'detection_node = object_detection_pkg.detection_node:main'
        ],
    },
)
