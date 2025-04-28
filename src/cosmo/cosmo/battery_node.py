import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import BatteryState

import smbus3 as smbus
from cosmo.rpio import InputPin, OutputPin, OutputPWMPin
import json

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


# The Battery management chip is called the MAX17263, the code guide is below:
# https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf


class BatteryNode(Node):

    battery_params = {  # Read here and find the registers to find more detailed information: https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf
        "DesignCap": 0,  # The expected design capacity of the cell.
        "IchgTerm": 0,  # Termination current, when charging current is between 0.125 * Ichgterm and 1.25 * Ichgterm it will terminate.
        "VEmpty": 0xA5, # VEmpty is considered 0%, the empty voltage target. Default is 3.3V 
        #For VEmpty, the docs are odd. It says a 10mV resolution with 8 bits, which is 256 levels(??), it probably is wrong at it is actually 20mV per level. 0-5.11V 
        "FullSOCThr": 0
        }
    
    model_cfg = 0b1000010000000000  # refer to the ModelCFG page. Bit 10 and 15 are set (hopefully the correct endian)

    def __init__(self):  # initialising the ROS2 node
        super().__init__("battery_node")

        self.i2c_address = 0x02 # placeholder address 
        self.bus = smbus.SMBus("/dev/i2c-1")  # I believe that this is the /dev/ bus number. This is different
        # from the I2C address, which will be found once I hook the MAX17263 chip up with the Pi.
 
        timer_frequency = 0.5 # frequency of battery updates (Hz)
        self._timer = self.create_timer(1/timer_frequency, self.get_battery_state)  # for updating the battery status  
        self._register_timeout = self.create_rate(1e-3, self.get_clock())  # for timeouts for verifying registers
        self._timeout = self.create_rate(1e-2, self.get_clock())  # for timeouts for initialising the chip
        self._save_charge = self.create_timer(10, self._save_params)  # every 10s, check bit 6 of the Cycles register to save charge parameters. 

        self.battery_pub = self.create_publisher(msg_type=BatteryState, topic="/battery/output", qos_profile=QoS)
        self.battery_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/battery/input", qos_profile=QoS, callback=self._battery_callback)


    def _init_chip(self):  # initialising the IC chip on powerup.
        STATUS_POR_BIT = self._read_register(0x00) & 0x0002

        if STATUS_POR_BIT == 0: 
            self.get_battery_state()
            return

        HibCFG = self._read_register(0xBA)  # Store original HibCFG values
        self._write_register(0x60, 0x90)  # exiting hibernate mode step 1
        self._write_register(0xBA, 0x0)  # step 2
        self._write_register(0x60, 0x0)  # step 3

        msg = f"Timeout: the IC MAX17263 did not clear the DNR bit in the Fstat register within 10 seconds."
        self._wait(0x3D, 0x01, msg)
        
        self._write_register(0xDB, self.model_cfg)  # Set the modelcfg register

        # Poll ModelCFG.Refresh(highest bit) until it becomes 0 to confirm IC completes model loading        
        msg_2 = f"Timeout: the ModelCFG.Refresh bit (bit 15) confirming model loading was not cleared."
        self._wait(0xDB, 0x8000, msg_2)

        with open("battery_parameters.json", "r") as fs:
            json_data = json.load(fs)

        self._write_register(0x38, self._hex_to_dec(json_data["RComp0"]))
        self._write_register(0x39, self._hex_to_dec(json_data["TempCo"]))

         # THE DOCUMENTATION IS SHITE, WHAT THE FUCK IS A QRTABLE RAHHHHHH
        # Basically it doesn't explain at all what is a QRTable and which characterisation params to write
        # so I'll just ignore it for now. We need to figure out tf it means though. 

        self._write_register(0xBA, HibCFG) # restore original HibCFG values. 


    @staticmethod
    def _hex_to_dec(hex):
        return int(hex, 16)


    def _wait(self, register, bit_mask, error_msg, timeout_seconds=10):
        timeout_count = 0  # 1000 * 1e-2 = 10 seconds
        while self._read_register(register) & bit_mask != 0:
            self._timeout.sleep()
            _timeout_count += 1
            if _timeout_count * 1e-2 > timeout_seconds:
                self.get_logger().error(error_msg)
                break

    def _read_register(self, register):  # uint8 register value
        register_value = self.bus.read_word_data(self.i2c_address, register)
        return register_value


    def _write_register(self, register, value):  # uint8 reg, uint16 value
        self.bus.write_word_data(self.i2c_address, register, value)


    def _write_and_verify_register(self, register, value): # uint8 reg, uint16 value
        attempts = 0
        
        while True:
            self._write_register(register, value)
            self._register_timeout.sleep()
            if value != self._read_register(register):
                attempts += 1
            elif attempts >= 3:
                self.get_logger().error(f"Write Error: failed to write data ({value}) to register {register}.")
                break
            else:
                break


    def _save_params(self): 
        if self._read_register(0x17) & 0b100000:  # check bit 6. COULD BE WRONG IF THE ENDIAN IS WRONG. 
            SAVED_RCOMP0 = self._read_register(0x38)  # characterisation information for open circuit operation
            SAVED_TempCo = self._read_register(0x39)  # temperature compensation information for RComp0 reg
            SAVED_FullCapRep = self._read_register(0x10) # reports the full capacity that goes with RepCap, generally used for reporting to the GUI
            SAVED_Cycles = self._read_register(0x17)  # Total number of charge/discharge cycles of the cell that has occured. 
            # Cycles has a full range of 0 to 655.35 cycles with a 1% LSB. 
            SAVED_FullCapNom = self._read_register(0x23)  # The full discharge capacity compensated according to present conditions. 

        json_data = {
            "RComp0": SAVED_RCOMP0,
            "TempCo": SAVED_TempCo,
            "FullCapRep": SAVED_FullCapRep,
            "Cycles": SAVED_Cycles,
            "FullCapNom": SAVED_FullCapNom
                     }

        with open("battery_parameters.json", "w") as fs:
            json.dump(json_data, fs)

    def _battery_callback(self, msg):  # NEED TO UPDATE THIS AND HAVE A STORAGE OBJ. 
        self.get_battery_state()

        # https://learn.adafruit.com/scanning-i2c-addresses/raspberry-pi 
        # The I2C address can be found here

        # Some form of data processing here to interpret the data, then convert it to a stdmsg type to send over ROS. 

        msg = BatteryState() # Create a message of this type 
        msg.voltage = self.battery_voltage # set this to the battery voltage
        msg.percentage = self.percent_charge_level # and this to the percentage charge level of the battery

        self.battery_pub.publish(msg) # Publish BatteryState message 

    def get_battery_state(self):
        RepCap = self._read_register(0x05)  # Capacity in mAh  
        RepSOC = self._read_register(0x06)  # Capacity as a percentage, 256 levels. Round to nearest int. 
        # SOC means State of Charge. 
        TTE = self._read_register(0x11)  # Time to Empty value, each level is 5.25 seconds. 
        
   

def main(args=None):
    rclpy.init(args=args)
    _battery_node = BatteryNode()
    rclpy.spin(_battery_node)
    _battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()