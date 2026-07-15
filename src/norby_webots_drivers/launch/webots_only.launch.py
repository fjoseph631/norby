import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    ws_dir = get_package_share_directory('norby_webots_drivers')
    world_path = os.path.join(ws_dir, 'worlds', 'apartment.wbt')

    webots = WebotsLauncher(world=world_path)

    return LaunchDescription([
        webots,
    ])
