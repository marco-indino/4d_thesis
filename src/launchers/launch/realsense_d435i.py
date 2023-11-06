'''
Bring up the RealSense D435i camera.
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_on_arg = DeclareLaunchArgument(name="rviz_on", default_value="True")

    rviz_config = os.path.join(
        get_package_share_directory('launchers'), 'rviz', 'd435i.rviz'
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

    camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch/rs_launch.py'
                ])
            ]),
            launch_arguments={
                # for a complete list of the launch arguments, take a look at realsense2_camera/launch/rs_launch.py
                'pointcloud.enable': 'true',
                'camera_name': 'd435i',
                'serial_no': "'944622073971'"
            }.items()
    )

    return LaunchDescription([
        camera_node,
        rviz_on_arg, rviz_node
    ])
