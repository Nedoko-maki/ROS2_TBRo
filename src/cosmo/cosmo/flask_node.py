import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cosmo.flask_app.app as flask_app

QoS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, # Keep only up to the last 10 samples
    depth=10,  # Queue size of 10
    reliability=ReliabilityPolicy.BEST_EFFORT,  # attempt to deliver samples, 
    # but lose them if the network isn't robust
    durability=DurabilityPolicy.VOLATILE, # no attempt to persist samples. 
    # deadline=
    # lifespan=
    # liveliness=
    # liveliness_lease_duration=

    # refer to QoS ros documentation and 
    # QoSProfile source code for kwargs and what they do
)


class FlaskNode(Node):

    bridge = CvBridge()

    def __init__(self):
        super().__init__("model_node")

        self.camera_pub = self.create_publisher(msg_type=Image, topic="/flask/output/camera_feed", qos_profile=QoS)
        self.control_pub = self.create_publisher(msg_type=Image, topic="/flask/output/commands", qos_profile=QoS)
        self.control_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/flask/input", qos_profile=QoS, callback=self._control_callback)

        flask_app.start_server()


    def _control_callback(self, msg):
        pass


    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    


def main(args=None):
    rclpy.init(args=args)
    __flask_node = FlaskNode()
    rclpy.spin(__flask_node)
    __flask_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()