from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ai_model_node",
            namespace="model_nodes", # read docs for why this is what it is? avoiding namespace collisions I think.
            executable="ml_node",
            name="ml_node_1"
        )
        ]
    )