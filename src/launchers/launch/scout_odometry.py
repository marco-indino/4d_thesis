'''
Bring up the Scout robot.
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    rviz_on_arg = DeclareLaunchArgument(name="rviz_on", default_value="True")

    rviz_config = os.path.join(
        get_package_share_directory('launchers'), 'rviz', 'scout_odometry.rviz'
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

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('scout_base'),
                'launch/scout_base.launch.py'
            ])
        ]),
        launch_arguments={
            # for a complete list of the launch arguments, take a look at scout_base/launch/scout_base.launch.py
            'is_scout_mini': 'false'
        }.items()
    )

    return LaunchDescription([
        robot_node,
        rviz_on_arg, rviz_node
    ])
