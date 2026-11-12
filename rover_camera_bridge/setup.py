from setuptools import setup

package_name = 'rover_camera_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Camera board bridge for the ESP32 rover: video and IR sensors.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rover_camera_bridge = rover_camera_bridge.rover_camera_bridge:main',
        ],
    },
)
