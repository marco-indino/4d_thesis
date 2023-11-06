'''
Launch all the transforms linking the sensors' frames to base_link.
'''

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    os1_tf_node = Node(
                package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["-0.05", "0.0", "0.714", "0.0", "0.0", "0.0", "base_link", "os_sensor"]
                )

    d435i_tf_node = Node(
                package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0.03", "0.087", "0.557", "0.7854", "0.0", "1.5708", "base_link", "d435i_link"]
                )
    
    d435_tf_node = Node(
                package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0.03", "-0.087", "0.557", "-0.7854", "0.0", "-1.5708", "base_link", "d435_link"]
                )

    ld.add_action(os1_tf_node)
    ld.add_action(d435i_tf_node)
    ld.add_action(d435_tf_node)

    return ld
