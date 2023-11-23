from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "scout_2")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "ekf_w_gps.yaml")

    return LaunchDescription(
        [

            launch_ros.actions.Node(
                package="robot_localization",
                executable="test_robot_localization_estimator",
                name="robot_localization_listener",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odom/filtered","odometry/global"),
                            ("acceleration/filtered","accel/filtered")]
            ),

            # launch_ros.actions.Node(
            #     package="robot_localization",
            #     executable="filter_base-test",
            #     name="filter_base",
            #     output="screen",
            #     parameters=[rl_params_file, {"use_sim_time": True}],
            #     # remappings=[("odom/filtered","odometry/global"),
            #     #             ("acceleration/filtered","accel/filtered")]
            # ),

            launch_ros.actions.Node(
                package="robot_localization",
                executable="robot_localization_listener_node",
                name="robot_localization_listener",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odom/filtered","odometry/global"),
                             ("acceleration/filtered","accel/filtered")]
            ),
        ]
    )
