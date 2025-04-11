from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cosmo",
            namespace="cosmo", # read docs for why this is what it is? avoiding namespace collisions I think.
            executable="control_node",
            name="control_node"
        ),
        Node(
            package="cosmo",
            namespace="cosmo", # read docs for why this is what it is? avoiding namespace collisions I think.
            executable="flask_node",
            name="flask_node"
        ),
        Node(
            package="cosmo",
            namespace="cosmo", # read docs for why this is what it is? avoiding namespace collisions I think.
            executable="model_node",
            name="model_node"
        ),
        Node(
            package="cosmo",
            namespace="cosmo", # read docs for why this is what it is? avoiding namespace collisions I think.
            executable="motor_driver_node",
            name="motor_driver_node"
        )
        ]
    )