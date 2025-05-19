import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray, String
from sensor_msgs.msg import Image, BatteryState
from cv_bridge import CvBridge
from cosmo_msgs.msg import SystemInfo, SystemCommand

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
node_create_event = threading.Event()

class ControlNode(Node):

    bridge = CvBridge()

    def __init__(self, sleep_node):
        super().__init__("control_node")

        self.motor_pub = self.create_publisher(msg_type=SystemCommand, topic="/motor_driver/input", qos_profile=QoS)
        self.motor_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/motor_driver/output", qos_profile=QoS, callback=self._motor_callback)

        self.model_sub = self.create_subscription(msg_type=Image, topic="/model/output", qos_profile=QoS, callback=self._model_callback)
        
        self.flask_battery_pub = self.create_publisher(msg_type=BatteryState, topic="/flask/input/battery", qos_profile=QoS)
        self.flask_motor_pub = self.create_publisher(msg_type=String, topic="/flask/input/motor", qos_profile=QoS)
        self.flask_model_pub = self.create_publisher(msg_type=Image, topic='/flask/input/model', qos_profile=QoS)

        self.flask_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/flask/output/commands", qos_profile=QoS, callback=self._flask_callback)

        self.battery_pub = self.create_publisher(msg_type=SystemCommand, topic="/battery/input", qos_profile=QoS)
        self.battery_sub = self.create_subscription(msg_type=BatteryState, topic="/battery/output", qos_profile=QoS, callback=self._battery_callback)

        self.battery_data = None

        self.test_timer = self.create_timer(20, callback=self.test_motors)
        self.test_rate = sleep_node.create_rate(4)

        self._send_to_flask_timer = self.create_timer(1, callback=self._send_battery_data_to_flask)
        # self.test_motors()  ## some test code 

    def _send_battery_data_to_flask(self):
        if self.battery_data:
            self.flask_battery_pub.publish(self.battery_data)
        else:
            self.get_logger().debug(f"Battery data is empty! Check the battery_node for problems.")

    def _motor_callback(self, msg):
        pass  # receive data from motors

    def _flask_callback(self, msg):
        pass  # receieve commands from the flask app

    def _battery_callback(self, msg):
        # self.battery_data = {  # technically I'm creating a new dict every time this runs?
            # "voltage": msg.voltage,  # voltage
            # "percentage": msg.percentage,  # charge percentage normalised from 0 to 1
            # "current": msg.current,  # discharge current, negative when discharging. 
            # "charge": msg.charge,  # charge in Ah.
            # "capacity": msg.capacity,  # capacity in Ah. 
            # "design_capacity": msg.design_capacity  # Design capacity                 
                            #  }   
        self.battery_data = msg

    def _model_callback(self, msg):
        pass  # receive ML-processed images from the model 

    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
    def _convert_cv2_to_imgmsg(self, msg):
        return self.bridge.cv2_to_imgmsg(msg, "passthrough")
    
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
                msg.command, msg.value = cmd[0]
            else:
                msg.command, msg.value = cmd[0], None

            self.test_rate.sleep()
            self.motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    

    global sleep_node
    sleep_node = rclpy.create_node("sleep_node")  # There is a possibility that this being accessed by
    # different nodes could cause major problems. 

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
    
if __name__ == "__main__":
    main()