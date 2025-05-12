import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from queue import Queue

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
        super().__init__("flask_node")

        self.camera_pub = self.create_publisher(msg_type=Image, topic="/flask/output/camera_feed", qos_profile=QoS)
        self.control_pub = self.create_publisher(msg_type=String, topic="/flask/output/commands", qos_profile=QoS)
        self.control_sub = self.create_subscription(msg_type=String, topic="/flask/input", qos_profile=QoS, callback=self._control_callback)
        
        self.command_queue = Queue() # pass this into the server so it can send back data...
        # Can we send a function/method instead to run to pass back data?
        self.image_queue = Queue()

        # self.poll_timer = self.create_timer(period, callback=self._poll_commands)

        #url = 
        # flask_app.start_server()
        # self.vcap = cv2.VideoCapture(url)

    def _receive_commands(self, command):
        # receive commands from the flask app and publish to the /flask/output/commands topic
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)

    def _control_callback(self, msg):
        # send back relevant data metrics from battery, motor, temps, etc. 
        pass

    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    


def main(args=None):
    rclpy.init(args=args)
    _flask_node = FlaskNode()
    try:
        rclpy.spin(_flask_node)
    except KeyboardInterrupt:
        _flask_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _flask_node.destroy_node()
        rclpy.try_shutdown()