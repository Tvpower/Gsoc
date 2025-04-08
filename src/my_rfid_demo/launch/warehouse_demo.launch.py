import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    #Get the launch directory
    pkg_dir = get_package_share_directory('my_rfid_demo')

    #launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    #robot URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'warehouse_robot.urdf.xacro')

    #parse XACRO
    robot_description = Command(['xacro ', urdf_file])

    #set world file path
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')

    #include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    #bridge for all needed topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/dummy_sensor@std_msgs/msg/String@gz.msgs.StringMsg',
            # Add cmd_vel bridge (both directions)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )

    #listener node
    listener = Node(
        package='demo_nodes_cpp',
        executable='listener',
        remappings=[('/chatter', '/dummy_sensor')],
        output='screen'
    )

    #robot state config
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    return LaunchDescription([
        gazebo,
        bridge,
        listener,
        robot_state_publisher
    ])