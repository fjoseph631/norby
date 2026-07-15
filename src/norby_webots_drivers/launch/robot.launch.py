import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    ws_package = 'norby_webots_drivers'
    ws_dir = get_package_share_directory(ws_package)
    world_path = os.path.join(ws_dir, 'worlds', 'apartment.wbt')
    robot_description_path = os.path.join(ws_dir, 'urdf', 'TurtleBot3Burger.urdf')

    webots = WebotsLauncher(world=world_path)

    controllers_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ws_dir, 'launch', 'controllers.launch.py'),
        ),
    )

    with open(robot_description_path, encoding='utf-8') as urdf_file:
        urdf = urdf_file.read()

    slam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ws_dir, 'launch', 'slam.launch.py'),
        ),
        launch_arguments={'robot_description': urdf}.items(),
    )

    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ws_dir, 'launch', 'navigation.launch.py'),
        ),
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=webots,
        nodes_to_start=[controllers_launch, slam_launch, navigation_launch],
    )

    return LaunchDescription([
        webots,
        waiting_nodes,
    ])
