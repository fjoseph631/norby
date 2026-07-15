import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ws_package = "norby_webots_drivers"
    slam_package = "slam_toolbox"
    ld = LaunchDescription()
    ws_dir = get_package_share_directory(ws_package)
    slam_dir = get_package_share_directory(slam_package)


    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    pcl_to_laser_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node', name='pcl_node',
        remappings=[
            ('cloud_in', 'base_cloud'),
            ('scan', 'laser_scan')
        ])

    slam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'params_file': os.path.join(ws_dir, 'config', 'slam_config.yml'),
        }.items(),
    )
    ld.add_action(pcl_to_laser_node)
    ld.add_action(footprint_publisher)
    return launch.LaunchDescription([
        ld,
        slam_launch
    ])

