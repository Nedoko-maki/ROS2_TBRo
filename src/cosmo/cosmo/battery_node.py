import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import BatteryState

import RPi.GPIO as GPIO 
import smbus3 as smbus

# EXTREMELY IMPORTANT LINKS FOR LOW LEVEL INTERACTIONS

# https://www.waveshare.com/wiki/Raspberry_Pi_Tutorial_Series:_I2C
# https://www.isopensource.com/blog/pcf8574-python-raspberry5.html
# https://pypi.org/project/smbus3/
# https://smbus2.readthedocs.io/en/latest/#smbus2.i2c_msg


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

# Might be a good idea to change the QoS settings for battery data. (Important to keep all data? Make sure all all data is received?)



class BatteryNode(Node):

    def __init__(self):
        super().__init__("battery_node")

        self.bus = smbus.SMBus(0)  # guessing the i2c interface path, will check once enabled and put here

        timer_frequency = 0.5 # frequency of battery updates (Hz)
        self.timer = self.create_timer(1/timer_frequency, self.get_battery_state)    

        self.battery_pub = self.create_publisher(msg_type=BatteryState, topic="/battery/output", qos_profile=QoS)
        self.battery_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/battery/input", qos_profile=QoS, callback=self._battery_callback)



    def _battery_callback(self, msg):
        pass


    def get_battery_state(self):
        bus_data = self.bus.read_byte_data("i2c_address in integer", "register to read, in integer")
        
        # https://learn.adafruit.com/scanning-i2c-addresses/raspberry-pi 
        # The I2C address can be found here

        # Some form of data processing here to interpret the data, then convert it to a stdmsg type to send over ROS. 

        msg = BatteryState() # Create a message of this type 
        msg.voltage = self.battery_voltage # set this to the battery voltage
        msg.percentage = self.percent_charge_level # and this to the percentage charge level of the battery

        self.battery_pub.publish(msg) # Publish BatteryState message 
        
   

def main(args=None):
    rclpy.init(args=args)
    _battery_node = BatteryNode()
    rclpy.spin(_battery_node)
    _battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()