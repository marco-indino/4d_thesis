import os
import random
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess , IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import ( OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro 



def generate_launch_description():
    
    package_name = 'scout_2'
    world_file = 'labirinto_test.world'
    pkg_world = get_package_share_directory(package_name)
    robot_localization_file_path = os.path.join(pkg_world, 'config/ekf_w_gps.yaml')

    #use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = os.path.join(
        get_package_share_directory('scout_2'), 'rviz', 'main.rviz'
    )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),)
    

    # package_path = os.path.join(get_package_share_directory('scout_2'))

    # xacro_file = os.path.join( package_path,'urdf','scout.urdf')

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description':doc.toxml()}

    doc = xacro.process_file('/home/marco/4d_thesis/src/scout_2/urdf/scout_prova.xacro')
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    

    #entity_name = robot_base_name #+"-"+str(int(random.random()*100000))
    
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        name = 'spawn_entity',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scout2'],
                        output='screen')    

    # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[robot_localization_file_path],
        remappings=[('odometry/filtered', 'odometry/local')],
        )
    
    #Start robot localization using an Extended Kalman filter...map->odom transform
    start_robot_localization_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[robot_localization_file_path],
        remappings=[('odometry/filtered', 'odometry/global')],
        )
    
    # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[robot_localization_file_path],
        remappings=[("imu/data", "imu/data"),
                    ("gps/fix", "gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global")],
    )

    rviz_node = Node(
        package= "rviz2",
        executable= "rviz2",
        arguments=['-d', rviz_config],

    )

    
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        start_robot_localization_local_cmd,
        start_robot_localization_global_cmd,
        start_navsat_transform_cmd,
        rviz_node,
 

    ])
