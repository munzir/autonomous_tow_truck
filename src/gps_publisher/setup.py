from setuptools import setup
from setuptools import find_packages
from glob import glob

package_name = 'gps_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='lyeba',
    maintainer_email='mq06812@st.habib.edu.pk',
    description='ROS2 GPS Publisher Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'gps_publisher = gps_publisher.gps_publisher_node:main',
            'gps_publisher_static = gps_publisher.gps_publisher_static:main',
	    'gps_publisher = gps_publisher.gps_publisher:main',
            'gps_publisher_realtime = gps_publisher.gps_publisher_realtime:main',
        ],
    },
)

