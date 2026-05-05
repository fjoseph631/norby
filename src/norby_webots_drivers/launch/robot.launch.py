import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    ws_package = "norby_webots_drivers"
    ws_dir = get_package_share_directory(ws_package)
    webots = WebotsLauncher(
        world=os.path.join(ws_dir, 'worlds', 'apartment.wbt'))
    webots_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ws_dir + '/launch/webots.launch.py'),
        launch_arguments={
            'world': '/home/frantzcito/my_project/worlds/my_first_sim.wbt'}.items()
    )

    controllers_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ws_dir + '/launch/controllers.launch.py'),
        launch_arguments={
            'world': '/home/frantzcito/my_project/worlds/my_first_sim.wbt'}.items()
    )
    robot_description_path = os.path.join(ws_dir, 'urdf', "TurtleBot3Burger.urdf")
    urdf = open(robot_description_path).read()
    slam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ws_dir + '/launch/slam.launch.py'),
        launch_arguments={'robot_description': urdf}.items()
    )

    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ws_dir + '/launch/navigation.launch.py'),
        launch_arguments={}.items()
    )
    waiting_nodes = WaitForControllerConnection(
        target_driver=webots,
        nodes_to_start=[controllers_launch, slam_launch, navigation_launch]
    )
    # return launch.LaunchDescription([
    #     webots, 
    #     waiting_nodes
    # ])
    return launch.LaunchDescription([
        controllers_launch, slam_launch, navigation_launch
    ])
