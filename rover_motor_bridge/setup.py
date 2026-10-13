from setuptools import find_packages, setup

package_name = 'rover_motor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rover_wheel_test.launch.py',
            'launch/rover_motor_bridge.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='you@example.com',
    description='Bridges ROS 2 /cmd_vel Twist messages to the ESP32 rover UDP motor protocol.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_motor_bridge = rover_motor_bridge.rover_motor_bridge:main',
            'rover_wheel_test = rover_motor_bridge.rover_wheel_test:main',
        ],
    },
)
