import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    gps_wpf_dir = get_package_share_directory(
        "scout_2")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "sonoma_raceway_modi.world")

    # urdf = os.path.join(gps_wpf_dir, 'urdf', 'scout.urdf')
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()


    doc = xacro.process_file('/home/marco/4d_thesis/src/scout_2/urdf/scout_prova.xacro')
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    # models_dir = os.path.join(gps_wpf_dir, "models")
    # models_dir += os.pathsep + \
    #     f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    # set_gazebo_model_path_cmd = None

    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + \
    #         os.pathsep + models_dir
    #     set_gazebo_model_path_cmd = SetEnvironmentVariable(
    #         "GAZEBO_MODEL_PATH", gazebo_model_path)
    # else:
    #     set_gazebo_model_path_cmd = SetEnvironmentVariable(
    #         "GAZEBO_MODEL_PATH", models_dir)

    #set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}])
    
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        name = 'spawn_entity',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'scout2','-x','2.0','-y','-2.5','-z','0.3'],
                        output='screen') 

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set gazebo up to find models properly
    # ld.add_action(set_gazebo_model_path_cmd)
    # ld.add_action(set_tb3_model_cmd)

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    #spawn_entity
    ld.add_action(spawn_entity)

    return ld
