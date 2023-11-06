'''
Bring up the Bunker Mini and the Ouster OS1 (to be used together with the autonomous navigation code).
'''

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bunker_odometry_node = GroupAction(
        actions=[
            SetRemap(src='/odom', dst='/integrated_to_init'),
            IncludeLaunchDescription(
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
        ]
    )

    scout_odometry_node = GroupAction(
        actions=[
            SetRemap(src='/odom', dst='/integrated_to_init'),
            IncludeLaunchDescription(
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
        ]
    )

    os1_node = GroupAction(
        actions=[
            SetRemap(src='/ouster/points', dst='/velodyne_cloud_registered'),
            IncludeLaunchDescription(
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
        ]
    )

    return LaunchDescription([
        # bunker_odometry_node,
        scout_odometry_node,
        os1_node,
    ])
