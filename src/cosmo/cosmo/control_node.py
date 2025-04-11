import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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


class ControlNode(Node):

    bridge = CvBridge()

    def __init__(self):
        super().__init__("control_node")

        self.motor_pub = self.create_publisher(msg_type=Int16MultiArray, topic="/motor_driver/input", qos_profile=QoS)
        self.motor_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/motor_driver/output", qos_profile=QoS, callback=self._motor_callback)

        self.model_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/model/output", qos_profile=QoS, callback=self._model_callback)
        
        self.flask_pub = self.create_publisher(msg_type=Int16MultiArray, topic="/flask/input", qos_profile=QoS)
        self.flask_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/flask/output/commands", qos_profile=QoS, callback=self._control_callback)


    def _motor_callback(self, msg):
        pass

    def _control_callback(self, msg):
        pass

    def _model_callback(self, msg):
        pass

    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    


def main(args=None):
    rclpy.init(args=args)
    _cosmo_node = ControlNode()
    rclpy.spin(_cosmo_node)
    _cosmo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()