import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
import RPi.GPIO as GPIO

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

# Not bad starting point to look at: https://wiki.ros.org/dynamixel_controllers/Tutorials/ConnectingToDynamixelBus
# GPIO documentation: https://sourceforge.net/p/raspberry-gpio-python/wiki/Examples/ 

GPIO.setmode(GPIO.BOARD)


class MotorDriverNode(Node):

    pins = {
        "MOTOR_A": {"IN1": 32, "IN2": 12, "NFAULT": 37, "SNSOUT": 10},
        "MOTOR_B": {"IN1": 32, "IN2": 12, "NFAULT": 16, "SNSOUT": 18},
        "MOTOR_C": {"IN1": 35, "IN2": 33, "NFAULT": 13, "SNSOUT": 15},
        "MOTOR_D": {"IN1": 35, "IN2": 33, "NFAULT": 29, "SNSOUT": 31},
        "NSLEEP": 40
                        }

    def __init__(self):
        super().__init__("motor_driver_node")

        self.control_pub = self.create_publisher(msg_type=Int16MultiArray, topic="/motor_driver/output", qos_profile=QoS)
        self.control_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/motor_driver/input", qos_profile=QoS, callback=self._control_callback)

        for motor_controller in self.pins:
            if motor_controller != "NSLEEP":
                for pin_name, pin in motor_controller.items():
                    match pin_name:
                        case "IN1": GPIO.setup(pin, GPIO.OUT)
                        case "IN2": GPIO.setup(pin, GPIO.OUT)
                        case "NFAULT": GPIO.setup(pin, GPIO.IN)
                        case "SNSOUT": GPIO.setup(pin, GPIO.IN)

        GPIO.setup(self.pins["NSLEEP"], GPIO.OUT)
        GPIO.output(self.pins["NSLEEP"], GPIO.HIGH)  # Device sleep mode: pull logic low. Otherwise set logic high. 

        # TI documentation on the motor drivers: https://www.ti.com/lit/ds/symlink/drv8701.pdf



    def _control_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    _motor_node = MotorDriverNode()
    rclpy.spin(_motor_node)
    _motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()