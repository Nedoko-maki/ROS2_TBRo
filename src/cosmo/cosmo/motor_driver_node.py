import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
import gpiozero as gpio
import cosmo.rpio as rpio

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


# RPi.GPIO is not available for RPi5 since the new Pi uses a custom RP1 chip instead of the previous interfacing chip.
# The two new contenders are gpiozero and gpiod. After some research there is evidence to show that gpiod is nearly 3x
# the polling rate, however for our purposes that may be excessive so we are 
# using the officially recommended library gpiozero instead.

# https://gpiozero.readthedocs.io/en/latest/ 

InputPin = gpio.DigitalInputDevice
OutputPin = gpio.DigitalOutputDevice 
OutputPWMPin = gpio.PWMOutputDevice


class MotorDriverNode(Node):

    pins = {
        "MOTOR_A": {"IN1": 12, "IN2": 18, "NFAULT": 26, "SNSOUT": 15},
        "MOTOR_B": {"IN1": 12, "IN2": 18, "NFAULT": 23, "SNSOUT": 24},
        "MOTOR_C": {"IN1": 19, "IN2": 13, "NFAULT": 27, "SNSOUT": 22},
        "MOTOR_D": {"IN1": 19, "IN2": 13, "NFAULT": 5, "SNSOUT": 6},
        "NSLEEP": 21
                        }  # As per spec, this is the GPIO pin IDs rather than the physical pin numbers. 

    pwm_freq = 100 # Not defined properly yet with calculations for the Motor Bridge IC. 

    def __init__(self):
        super().__init__("motor_driver_node")

        self.control_pub = self.create_publisher(msg_type=Int16MultiArray, topic="/motor_driver/output", qos_profile=QoS)
        self.control_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/motor_driver/input", qos_profile=QoS, callback=self._control_callback)

        for motor in self.pins:
            if motor != "NSLEEP":
                for pin_name, pin in self.pins[motor].items():
                    match pin_name:
                        case "IN1": self.pins[motor][pin_name] = rpio.set_pin(pin, OutputPWMPin, initial_value=False, frequency=self.pwm_freq)
                        case "IN2": self.pins[motor][pin_name] = rpio.set_pin(pin, OutputPWMPin, initial_value=False, frequency=self.pwm_freq)
                        case "NFAULT": self.pins[motor][pin_name] = rpio.set_pin(pin, InputPin)
                        case "SNSOUT": self.pins[motor][pin_name] = rpio.set_pin(pin, InputPin)

        # Device sleep mode: pull logic low for sleep mode. Otherwise set logic high. 
        self.pins["NSLEEP"] = rpio.set_pin(self.pins["NSLEEP"], OutputPin, active_high=True, initial_value=True) 

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