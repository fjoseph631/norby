import os
from setuptools import setup
from glob import glob
package_name = 'norby_webots_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    #package_dir={'camera_calibration':'../camera_calibration'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name,'config'), glob('config/*.yml')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.wbt'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frantzcito',
    maintainer_email='frantzcito@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'radar_node  = norby_webots_drivers.radar_node:main',
           'radar_publisher_node  = norby_webots_drivers.radar_publisher_node:main',
           'camera_node = norby_webots_drivers.camera_node:main',
           'gps_node = norby_webots_drivers.gps_node:main',
           'sonar_node = norby_webots_drivers.sonar_node:main',
           'imu_node = norby_webots_drivers.imu_node:main',
           'lidar_node = norby_webots_drivers.lidar_node:main',
           'object_tracker_node = norby_webots_drivers.object_tracker_node:main'
        ],
    },
)
