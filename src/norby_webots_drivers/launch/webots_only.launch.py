import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    ws_package = "norby_webots_drivers"
    webots_package = "webots_ros2_driver"
    webots_dir = get_package_share_directory(webots_package)
    ws_dir = get_package_share_directory(ws_package)
    webots = WebotsLauncher(
        world=os.path.join(ws_dir, 'worlds', 'apartment.wbt'))
    webots_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            webots_dir + '/launch/robot_launch.py'),
        launch_arguments={
            'world': '/home/frant/norby/worlds/apartment.wbt'}.items()
    )
    return launch.LaunchDescription([
        webots
    ])
