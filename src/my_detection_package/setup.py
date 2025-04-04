from setuptools import find_packages, setup

package_name = 'my_detection_package'

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
    maintainer='arfah',
    maintainer_email='arfah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'object_detection_node = my_detection_package.detection_node:main',
        'headlights_talker = my_detection_package.headlights_talker:main',
        'headlights_listener = my_detection_package.headlights_listener:main',
        ],
    },
)
