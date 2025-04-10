import rclpy 
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from ai_model_node.model import PytorchModel, test_read
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



# Thoughts: how to implement many models? We will be running a few at the same
# time:
# - Depth Estimation 
# - Risk assessment of hexagon
# - Driving?
# - Arm control

# Proposal 1: we implement a node that will handle ALL models and will require
#             a preprocessing node to manage the information given to the node
# Proposal 2: we implement a node PER MODEL, and each gets their input directly
#             piped into them and is handled in a node-by-node basis. 

# Threads? How will the AI hat handle that?
# TODO:
# - implement a single model node that takes in input from a preprocessing node 
#   and gives output to a post-processing node




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
                 

            
class MachineLearningNode(Node):

    bridge = CvBridge()

    def __init__(self, node_name, sub_datatype, pub_datatype):
        super().__init__(node_name)

        self.output_publisher = self.create_publisher(msg_type=pub_datatype, topic="/model/output", qos_profile=QoS)
        self.input_subscriber = self.create_subscription(msg_type=sub_datatype, topic="/model/input", qos_profile=QoS, callback=self._callback)

        # self.model = PytorchModel()  # need to insert your model class with all the layers and convs, as well as the path to the weights. 
        # self.model.load_model()


        # self.get_logger().info(f"Can import! {type(PytorchModel), PytorchModel.__dict__}")

    def _callback(self, msg):
        pass


    def _convert_imgmsg_to_cv2(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    # passthrough is the encoding format. Refer to https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
    # for alternatives





def main(args=None):
    rclpy.init(args=args)
    
    _ml_node = MachineLearningNode("ml_node", Image, Image)

    _ml_node.get_logger().info(test_read("./model_data/test.txt"))

    rclpy.spin(_ml_node)

    _ml_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()