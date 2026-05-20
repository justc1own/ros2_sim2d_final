import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Get the path to the URDF file
    pkg_path = get_package_share_directory('ros2_sim2d')
    xacro_file = os.path.join(pkg_path, 'urdf', 'sim2d_robot.urdf.xacro')
    
    # Process the xacro file to generate the robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Get the path to the controllers configuration file
    controllers_config_file = os.path.join(pkg_path, 'config', 'sim2d_controllers.yaml')

    # RViz configuration file
    rviz_config_file = os.path.join(pkg_path, 'config', 'sim2d.rviz')

    # === Nodes to launch ===

    # 1. Robot State Publisher
    # Publishes the robot's state (transforms) to TF2 based on joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Controller Manager
    # The central node that loads and manages controllers and the hardware interface
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config_file],
        output="screen",
    )

    # 3. Joint State Broadcaster
    # Publishes the state of all joints
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 4. Differential Drive Controller
    # Takes Twist messages and converts them to wheel velocity commands
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # 5. RViz2
    # Visualization tool
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 6. Foxglove Bridge
    # Opens a websocket for Foxglove Studio to connect to
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        rviz_node,
        foxglove_bridge_node
    ])
