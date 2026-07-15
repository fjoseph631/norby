import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ws_package = "norby_webots_drivers"
    nav_package = 'nav2_bringup'
    ld = LaunchDescription()
    nav_dir = get_package_share_directory(nav_package)
    ws_dir = get_package_share_directory(ws_package)

    nav_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(ws_dir, 'config', 'nav_params.yml'),
        }.items(),
    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_node',
        parameters=[])

    ld.add_action(map_saver_node)
    return launch.LaunchDescription([
        ld,
        nav_launch
    ])
