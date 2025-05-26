import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from rosidl_runtime_py.convert import message_to_ordereddict

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from cosmo_msgs.msg import SystemInfo, SystemCommand, ErrorEvent
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
        # may not need this, and can just pull it directly from the model node. 
        
        self.control_pub = self.create_publisher(msg_type=SystemCommand, topic="/flask/output/commands", qos_profile=QoS)
        self.control_sub = self.create_subscription(msg_type=SystemInfo, topic="/flask/input", qos_profile=QoS, callback=self._control_callback)

        self.error_sub = self.create_publisher(msg_type=ErrorEvent, topic="/error_events", qos_profile=QoS)

        self.data = None

        self.test_timer = self.create_timer(10, callback=self._test_commands)
                       
        # self.command_queue = Queue() # pass this into the server so it can send back data...
        # # Can we send a function/method instead to run to pass back data?
        # self.image_queue = Queue()

        # self.poll_timer = self.create_timer(period, callback=self._poll_commands)

        # url = 
        # flask_app.start_server(receive_callback=_receive_command, send_callback=_get_data)
        # self.vcap = cv2.VideoCapture(url)

    def _test_commands(self):

        command_list= [
            ("write_register", "battery_node", 0x02, 0x02),
            ("read_register", "battery_node", 0x03),
            ("forward", "motor_driver_node", 1),
            ("forward_left", "motor_driver_node", 0.5)
        ]

        for cmd in command_list:
            self._receive_command(*cmd)

    def _receive_command(self, command, node, value1=None, value2=None):
        # receive commands from the flask app and publish to the /flask/output/commands topic
        msg = SystemCommand()
        msg.command = command
        msg.node_name = node
        
        msg.value1 = float(value1) if value1 else .0 # HAVE TO FILL ALL VALUES OF THE MSG OR IT BREAKS. 
        msg.value2 = float(value2) if value2 else .0 # ALSO HAS TO BE FLOAT32 VALUES. 

        self.control_pub.publish(msg)

    def _get_data(self):  # May run into some problems with this dict being written to as _get_data
        # gets called, causing race conditions. Might need a lock/mutex to fix this. 
        return self.data
    
    def _control_callback(self, msg):
        # send back relevant data metrics from battery, motor, temps, etc. 
        # assign data values from the ros2 message into a dict or something to store data
        # there will be another function for the flask server to call to retrieve these values
        self.data = message_to_ordereddict(msg) # refer to https://github.com/ros2/rosidl_runtime_py/blob/1979f566c3b446ddbc5c3fb6896e1f03ccbc6a27/rosidl_runtime_py/convert.py#L159-L176 
        # self.get_logger().info(str(self.data))

    def _convert_imgmsg_to_cv2(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, "passthrough")

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