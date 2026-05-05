import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
def generate_launch_description():
    ws_package = "norby_webots_drivers"
    ws_dir = get_package_share_directory(ws_package)
    ld = LaunchDescription()

    camera_node = Node(
        package = ws_package,
        executable = "camera_node"
    )
    robot_description_path = os.path.join(ws_dir, 'urdf', 'TurtleBot3Burger.urdf')
    with open(robot_description_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='publisher_node',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )
    ld.add_action(camera_node)
    ld.add_action(robot_state_publisher)
    #ld.add_action(tracker_node)
    ws_package = "norby_webots_drivers"
    
    ws_dir = get_package_share_directory(ws_package)
    robot_description_path = os.path.join(ws_dir, 'urdf', 'TurtleBot3Burger.urdf')
    ros2_control_params = os.path.join(ws_dir, 'config', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True,
             'set_robot_state_publisher': True},
              ros2_control_params
        ],
        output='screen',
        remappings=mappings,
        respawn=True
    )
    with open(robot_description_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    #robot_description = robot_description_path.read()
        # ROS control spawners

    controller_manager_timeout = ['--controller-manager-timeout', '600']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=[
            'diffdrive_controller',
            '--controller-manager-timeout', '120',  # 120 seconds
            '--switch-timeout', '100'               # 100 seconds for switch
        ]
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, 
                            #joint_state_broadcaster_spawner
                            ]

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners
    )
    delayed_wait = TimerAction(
    period=10.0,  # Delay in seconds
    actions=[ros_control_spawners[0]]
)
    return launch.LaunchDescription([
        turtlebot_driver,
        #controller_manager_node,
        ld,
        delayed_wait
    ])
