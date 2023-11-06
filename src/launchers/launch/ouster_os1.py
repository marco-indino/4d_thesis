'''
Bring up the Ouster OS1 lidar.
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    rviz_on_arg = DeclareLaunchArgument(name="rviz_on", default_value="True")

    rviz_config = os.path.join(
        get_package_share_directory('launchers'), 'rviz', 'os1.rviz'
    )

    rviz_node = LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz_on"))
        )
    ])

    '''
    lidar_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch/sensor.launch.xml'
            ])
        ]),
        launch_arguments={
            # for a complete list of the launch arguments, take a look at ouster_ros/launch/sensor.launch.xml
            'sensor_hostname': "'os1-991942000600.local'",
            'viz': "false"
        }.items()
    )
    '''

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch/driver.launch.py'
            ])
        ]),
        launch_arguments={
            # for a complete list of the launch arguments, take a look at ouster_ros/launch/sensor.launch.xml
            #'sensor_hostname': "'os1-991942000600.local'",
            'viz': "false"
        }.items()
    )

    return LaunchDescription([
        lidar_node,
        rviz_on_arg, rviz_node
    ])
