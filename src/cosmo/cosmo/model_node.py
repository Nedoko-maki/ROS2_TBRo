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


#
# 
# Doubt this reaches the final iteration of COSMO because of how processing intensive ML models can be. We haven't 
# got the budget to buy an AI hat and I can't shell out that money sadly. 
#
#

# Using this model with 2.2M params vs DepthAnythingV2's 24.8M on their smallest model. 
# https://github.com/noahzn/Lite-Mono?tab=readme-ov-file





class ModelNode(Node):

    bridge = CvBridge()

    def __init__(self):
        super().__init__("model_node")

        self.model_pub = self.create_publisher(msg_type=Int16MultiArray, topic="/model/output", qos_profile=QoS)
        self.flask_sub = self.create_subscription(msg_type=Image, topic="/flask/output/camera_feed", qos_profile=QoS, callback=self._model_callback)

        # import cosmo.model.test_simple as model_test
        # model_test.test_simple()

        # Maybe it is a good idea to run this in a separate thread? 


    def _model_callback(self, msg):
        cv_image = self._convert_cv2_to_imgmsg(msg)
        # cmap = model.infer(cv_image)
        # 

    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    


def main(args=None):
    rclpy.init(args=args)
    __model_node = ModelNode()
    try:
        rclpy.spin(__model_node)
    except KeyboardInterrupt:
        __model_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        __model_node.destroy_node()
        rclpy.try_shutdown()

