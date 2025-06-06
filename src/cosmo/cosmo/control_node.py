import rclpy
import rclpy.executors
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image, BatteryState
from cv_bridge import CvBridge
from cosmo_msgs.msg import SystemInfo, SystemCommand, ErrorEvent

import threading 

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

sleep_node = None

class ControlNode(Node):

    bridge = CvBridge()

    def __init__(self, sleep_node):
        super().__init__("control_node")

        self.motor_pub = self.create_publisher(msg_type=SystemCommand, topic="/motor_driver/input", qos_profile=QoS)
        self.motor_sub = self.create_subscription(msg_type=Float32MultiArray, topic="/motor_driver/output", qos_profile=QoS, callback=self._motor_callback)

        self.model_sub = self.create_subscription(msg_type=Image, topic="/model/output", qos_profile=QoS, callback=self._model_callback)
        
        self.flask_pub = self.create_publisher(msg_type=SystemInfo, topic="/flask/input", qos_profile=QoS)
        self.flask_sub = self.create_subscription(msg_type=SystemCommand, topic="/flask/output/commands", qos_profile=QoS, callback=self._flask_callback)

        self.battery_pub = self.create_publisher(msg_type=SystemCommand, topic="/battery/input", qos_profile=QoS)
        self.battery_sub = self.create_subscription(msg_type=BatteryState, topic="/battery/output", qos_profile=QoS, callback=self._battery_callback)

        self.error_sub = self.create_subscription(msg_type=ErrorEvent, topic="/error_events", qos_profile=QoS, callback=self._error_cb)

        self.battery_data, self.motor_data, self.model_data = None, dict(), None

        self.test_timer = self.create_timer(20, callback=self.test_motors)
        self.test_rate = sleep_node.create_rate(4)

        _send_system_data_period = 1
        self._send_to_flask_timer = self.create_timer(_send_system_data_period, callback=self._send_data_to_flask)
       

    def _send_data_to_flask(self):
        """Package up the ROS2 messages into the SystemCommand custom message and send to the 
        flask node. 
        """

        msg = SystemInfo()

        if self.battery_data: msg.battery_state = self.battery_data
        if self.motor_data: msg.motor_states = self.motor_data
        if self.model_data: msg.model_image = self.model_data

        self.flask_pub.publish(msg)

    def _motor_callback(self, msg):
        """Receive motor data in a Float32MultiArray ROS2 message.

        :param msg: ROS2 Message
        :type msg: Float32MultiArray
        """

        # # receive data from motors
        # tmp = message_to_ordereddict(msg)
        # # renaming the key the values are bound to so that the SystemInfo message
        # # accepts them. 
        # self.motor_data["motor_states"] = tmp.pop("data")
        self.motor_data = msg


    def _flask_callback(self, msg: SystemCommand):
        """Pipe ROS2 commands to their intended destination. 

        :param msg: ROS2 Message
        :type msg: SystemCommand
        """

        node_dest = msg.node_name

        match node_dest:
            case "battery_node" | "battery" | "bfg": 
                self.battery_pub.publish(msg)
            case "motor_driver_node" | "motor_driver" | "motor": 
                self.motor_pub.publish(msg)
            case "model_node": ...


    def _battery_callback(self, msg):
        """Receive battery BFG data in a BatteryState ROS2 message.

        :param msg: ROS2 Message
        :type msg: BatteryState
        """

        # self.battery_data = {  # technically I'm creating a new dict every time this runs?
            # "voltage": msg.voltage,  # voltage
            # "percentage": msg.percentage,  # charge percentage normalised from 0 to 1
            # "current": msg.current,  # discharge current, negative when discharging. 
            # "charge": msg.charge,  # charge in Ah.
            # "capacity": msg.capacity,  # capacity in Ah. 
            # "design_capacity": msg.design_capacity  # Design capacity                 
                            #  }
        self.battery_data = msg
        # self.get_logger().info(f"{message_to_ordereddict(msg)}")

    def _model_callback(self, msg):
        """Receive model image in an Image ROS2 message.

        :param msg: ROS2 Message
        :type msg: Image
        """
        # receive ML-processed images from the model 
        self.model_data = msg 

    def _error_cb(self, msg):
        """Process error messages from nodes.

        :param msg: ROS2 Message
        :type msg: ErrorEvent
        """

        node_name = msg.node
        error_name = msg.error
        tb = msg.traceback

        match error_name:
            case "BAT_NOT_PRESENT": ...
            case "BAT_I2C_FAIL": ...
            case "BAT_MODEL_CONFIG_NOT_SET": ...
            case "BAT_NODE_KILL": ...
            case "MOTOR_I2C_FAIL": ...
            case "MOTOR_NODE_KILL": ...
        # Do something here. 

    def _convert_cv2_to_imgmsg(self, img):
        """Convert a opencv2 image to a ROS2 Image message. 

        :param img: cv2 image
        :type img: numpy.ndarray, MatLike
        :return: ROS2 Message
        :rtype: Image
        """
        return self.bridge.cv2_to_imgmsg(img, "passthrough")
    
    def _convert_imgmsg_to_cv2(self, msg):
        """Convert a ROS2 Image message to a opencv2 image. 

        :param msg: ROS2 Message
        :type msg: Image
        :return: cv2 image
        :rtype: numpy.ndarray, MatLike
        """
        return self.bridge.imgmsg_to_cv2(msg, "passthrough")
    
    def test_motors(self):
        
        test_commands = [
            ["forward", 0.1],
            ["reverse", 0.2],
            ["motor_left", 0.3],
            ["motor_right", 0.25],
            ["coast"],
            ["brake"],
            ["sleep"],
            ["wake"]
                        ]

        for cmd in test_commands:
            msg = SystemCommand()
            if len(cmd) == 1:
                msg.command, msg.value1 = cmd[0], .0
            else:
                msg.command, msg.value1 = cmd

            msg.value2 = .0
            self.test_rate.sleep()
            self.motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    sleep_node = rclpy.create_node("control_sleep_node")  
    _cosmo_node = ControlNode(sleep_node=sleep_node)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(_cosmo_node)
    executor.add_node(sleep_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        _cosmo_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        sleep_node.destroy_node()
        _cosmo_node.destroy_node()
        rclpy.try_shutdown()  # this complains if it's called for some unknown reason. Do I require only 1 rclpy.shutdown() event?
        executor_thread.join()
