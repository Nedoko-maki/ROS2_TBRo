from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo
from launch.events import Shutdown
from launch.event_handlers import (OnShutdown)
from launch.substitutions import LocalSubstitution

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
            namespace="cosmo", 
            executable="flask_node",
            name="flask_node"
        ),
        Node(
            package="cosmo",
            namespace="cosmo", 
            executable="model_node",
            name="model_node"
        ),
        Node(
            package="cosmo",
            namespace="cosmo", 
            executable="motor_driver_node",
            name="motor_driver_node"
        ),
        Node(
            package="cosmo",
            namespace="cosmo", 
            executable="battery_node",
            name="battery_node"
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was shut down: ', LocalSubstitution('event.reason')]
                )]
            )
        )
        ]
    )