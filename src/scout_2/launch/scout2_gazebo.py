import os
import random
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess , IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import ( OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro 



def generate_launch_description():
    
    package_name = 'scout_2'
    world_file = 'labirinto_test.world'
    pkg_world = get_package_share_directory(package_name)

    rviz_config = os.path.join(
        get_package_share_directory('scout_2'), 'rviz', 'main.rviz'
    )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
            )
    
    
    
    # Position and orientation
    # [X, Y, Z]
    #position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    #orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "scout_2"

    # package_path = os.path.join(get_package_share_directory('scout_2'))

    # xacro_file = os.path.join( package_path,'urdf','scout.urdf')

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description':doc.toxml()}

    doc = xacro.process_file('/home/marco/gazebo_ws/src/scout_2/urdf/scout_prova.xacro')
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    

    entity_name = robot_base_name #+"-"+str(int(random.random()*100000))
    
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        name = 'spawn_entity',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scout_2'],
                        # arguments=['-entity',entity_name,
                        #             '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),'-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                        #             '-topic', '/robot_description'
                        #             ],
                        output='screen')    
    

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    rviz_node = Node(
        package= "rviz2",
        executable= "rviz2",
        arguments=['-d', rviz_config]

    )

    # world_arg = DeclareLaunchArgument(
    #       'world',
    #       default_value=[os.path.join(pkg_world, 'worlds', world_file),''],
    #       description='SDF world file')  
    
    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','--controller-manager','/joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_diff_drive_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','--controller-manager', '/diff_drive_controller'],
    #     output='screen'
    #)
    
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node,
        #diff_drive_spawner,
        #joint_broad_spawner,
        #world_arg,
        #load_joint_state_controller,
        # RegisterEventHandler(
        #     event_handler= OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[joint_broad_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler= OnProcessExit(
        #         target_action=joint_broad_spawner,
        #         on_exit=[diff_drive_spawner],
        #     )
        # ),

    ])
