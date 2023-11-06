'''
Bring up the Bunker Mini, the D435, the D435 and the Ouster OS1 (to be used as a demo of the sensors).
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('launchers'), 'rviz', 'demo.rviz'
    )

    rviz_node = LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])

    bunker_odometry_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/bunker_odometry.py'
            ])
        ]),
        launch_arguments={
            'rviz_on': 'false'
        }.items()
    )

    scout_odometry_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/scout_odometry.py'
            ])
        ]),
        launch_arguments={
            'rviz_on': 'false'
        }.items()
    )

    os1_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/ouster_os1.py'
            ])
        ]),
        launch_arguments={
            'rviz_on': 'false',
            'timestamp_mode': 'TIME_FROM_ROS_TIME'
        }.items()
    )

    d435i_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/realsense_d435i.py'
            ])
        ]),
        launch_arguments={
            'rviz_on': 'false'
        }.items()
    )

    d435_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/realsense_d435.py'
            ])
        ]),
        launch_arguments={
            'rviz_on': 'false'
        }.items()
    )

    sensors_transforms_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/sensors_transforms.py'
            ])
        ])
    )

    bunker_urdf_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/bunker_urdf_loader.py'
            ])
        ])
    )

    scout_urdf_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/scout_urdf_loader.py'
            ])
        ])
    )

    d435_urdf_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/d435_urdf_loader.py'
            ])
        ])
    )

    d435i_urdf_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/d435i_urdf_loader.py'
            ])
        ])
    )

    os1_urdf_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launchers'),
                'launch/os1_urdf_loader.py'
            ])
        ])
    )

   

    return LaunchDescription([
        rviz_node,
        # bunker_odometry_node,
        scout_odometry_node,
        os1_node,
        #d435i_node,
        #d435_node,
        #odom_to_baselink_tf_node,
        # odom_correction_node,
        sensors_transforms_node,
        # bunker_urdf_loader_node,
        scout_urdf_loader_node,
        #d435_urdf_loader_node,
        #d435i_urdf_loader_node,
        os1_urdf_loader_node,
        # odom_faker_node
    ])
