import rclpy 
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from picamera2 import Picamera2, Preview

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



#
#================================================================================
#
# DEPRECIATED, WE AREN'T USING THIS. KEEPING FOR ANY CODE THAT CAN BE REUSED. 
#
#================================================================================
#



class CameraNode(Node):

    bridge = CvBridge()

    def __init__(self):
        super.__init__("camera_node")

        self.output_publisher = self.create_publisher(msg_type=Image, topic="/camera_feed", qos_profile=QoS)

        self.camera = Picamera2()
        _camera_config = self.camera.create_preview_configuration()
        self.camera.configure(camera_config=_camera_config)
        self.camera.start_preview(Preview.DRM)
        self.camera.start()


    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")