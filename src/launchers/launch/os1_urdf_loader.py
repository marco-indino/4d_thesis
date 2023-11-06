'''
Load the URDF of the Ouster OS1 (generic model of a lidar).
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    urdf = os.path.join(
        get_package_share_directory('urdf_description'), 'urdf/sensors/os1.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    urdf_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_os1',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf],
        remappings=[('/robot_description', '/os1_description')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_os1',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf],
        remappings=[('/robot_description', '/os1_description')]
    )

    ld.add_action(urdf_node)
    ld.add_action(robot_state_publisher_node)

    return ld
