import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    #Get the launch directory
    pkg_dir = get_package_share_directory('my_rfid_demo')

    #set world file path
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')

    #include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
    )

    #bridge for the dummy sensor topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/dummy_sensor@std_msgs/msg/String@gz.msgs.StringMsg'],
        output='screen'
    )

    #listener node
    listener = Node(
        package='demo_nodes_cpp',
        executable='listener',
        remappings=[('/chatter', '/dummy_sensor')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        listener
    ])
