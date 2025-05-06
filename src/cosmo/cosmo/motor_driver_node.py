import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray, String
import cosmo.rpio as rpio
from cosmo.rpio import (
    DRV8701_Motor_LGPIO, 
    InputPin, 
    OutputPin, 
                        ) 


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
# the polling rate (~160kHz vs ~480kHz), however for our purposes that may be excessive so we are 
# using the officially recommended library gpiozero instead.

# https://gpiozero.readthedocs.io/en/latest/ 


class MotorDriverNode(Node):

    pins = {
        "MOTOR_A": {"IN1": 12, "IN2": 18, "NFAULT": 26, "SNSOUT": 15},
        "MOTOR_B": {"IN1": 12, "IN2": 18, "NFAULT": 23, "SNSOUT": 24},
        "MOTOR_C": {"IN1": 19, "IN2": 13, "NFAULT": 27, "SNSOUT": 22},
        "MOTOR_D": {"IN1": 19, "IN2": 13, "NFAULT": 5, "SNSOUT": 6},
        "NSLEEP": 21
                        }  # As per spec, this is the GPIO pin IDs rather than the physical pin numbers. 

    pwm_freq = 1e4 # Not defined properly yet with calculations for the Motor Bridge IC. Recommended in the DRV8701P datasheet to use 100kHz.
    # EDIT: Just found out lgpio doesn't like +30kHz on software PWM, so I'll run 10k for now. 

    

    def __init__(self):
        super().__init__("motor_driver_node")

        self.control_pub = self.create_publisher(msg_type=Int16MultiArray, topic="/motor_driver/output", qos_profile=QoS)
        self.control_sub = self.create_subscription(msg_type=String, topic="/motor_driver/input", qos_profile=QoS, callback=self._control_callback)
        self.motor_set_L = DRV8701_Motor_LGPIO(12, 18, pwm_frequency=self.pwm_freq)
        self.motor_set_R = DRV8701_Motor_LGPIO(19, 13, pwm_frequency=self.pwm_freq)

        for motor in [x for x in self.pins if x != "NSLEEP"]:

            for pin_name, pin in self.pins[motor].items():
                # self.get_logger().debug(f"{motor}, pin_name: {pin_name}, {pin}")
                
                match pin_name:
                    #case "IN1": self.pins[motor][pin_name] = rpio.set_pin(pin, OutputPWMPin, initial_value=False, frequency=self.pwm_freq)
                    #case "IN2": self.pins[motor][pin_name] = rpio.set_pin(pin, OutputPWMPin, initial_value=False, frequency=self.pwm_freq)
                    case "NFAULT": 
                        self.pins[motor][pin_name] = {"motor": motor, "pin": rpio.set_pin(pin, InputPin)}
                        self.pins[motor][pin_name]["pin"].when_deactivated = self._fault_detected  
                    case "SNSOUT": 
                        self.pins[motor][pin_name] = {"motor": motor, "pin": rpio.set_pin(pin, InputPin)}
                        self.pins[motor][pin_name]["pin"].when_deactivated = self._chopping_detected  

        # Device sleep mode: pull logic low for sleep mode. Otherwise set logic high. 
        self.pins["NSLEEP"] = rpio.set_pin(self.pins["NSLEEP"], OutputPin, active_high=True, initial_value=True) 

        # TI documentation on the motor drivers: https://www.ti.com/lit/ds/symlink/drv8701.pdf


    def _log(self, msg):
        self.get_logger().info(msg)


    def _fault_detected(self, device):
        self.get_logger().warn(f"Motor Fault: {device.motor_name} has pulled nFAULT low.")


    def _chopping_detected(self, device):
        self.get_logger().warn(f"Motor Fault: {device.motor_name} has pulled SNSOUT low. The drive current has hit the current chopping threshold.")


    def _control_callback(self, msg):
        
        # self.get_logger().info(f"Got message {msg}, {msg.data}")

        # From what I can discern, I don't need to think about SH1 and SH2, controlled by the PiHat. Page 14 on the datasheet.

        # VERY IMPORTANT WITH DATATYPES, look at the msg data type and their parameter names on the stdmsg ros2 docs, we use dot notation
        # to access them. e.g. String has a 'string data' .msg file. We access the data member with type string and use that to ferry
        # the data across topics and nodes. 

        # First instantiate the message type, e.g. msg = String(), and then access the member data, msg.data = "whatever". Bingo!
        
        # The message format goes as follows:
        # any of the below, such as 'forward', and if the case has a value inside, then follow up with a semicolon and the value after.
        # e.g. 'forward:0.1' 

        if ":" in msg.data:
            command, value = msg.data.split(":")
            value = float(value)
        else:
            command = msg.data

        match command:    
            case "forward":

                self.motor_set_L.forward(value)
                self.motor_set_R.forward(value)

            case "reverse":
                self.motor_set_L.backward(value)
                self.motor_set_R.backward(value)

            case "motor_left":
                self.motor_set_L.value = value
            
            case "motor_right":
                self.motor_set_R.value = value

            case "coast":
                self.motor_set_L.coast()
                self.motor_set_R.coast()

            case "brake":
                self.motor_set_L.brake()
                self.motor_set_R.brake()
            
            case "sleep": self.pins["NSLEEP"].off()
            case "wake": self.pins["NSLEEP"].on()


def main(args=None):
    rclpy.init(args=args)
    _motor_node = MotorDriverNode()
    rclpy.spin(_motor_node)
    _motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# TODO: 
#
# - add code to deal with individual motors being disabled and the rest of the motors still running. 
#
#