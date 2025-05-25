from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import (OnShutdown)
from launch.substitutions import LocalSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cosmo",
            namespace="cosmo", # read docs for why this is what it is? avoiding namespace collisions I think.
            executable="test_node",
            name="test_node"
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    LogInfo(
                        msg=['Launch was shut down: ', LocalSubstitution('event.reason')]
                        ),
                        # EmitEvent(event=Shutdown())
                        ]
            )
        )
        ]
    )