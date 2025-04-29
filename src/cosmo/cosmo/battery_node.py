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
# And relevant data sheets:
# https://www.analog.com/media/en/technical-documentation/data-sheets/MAX17263.pdf
# https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf


# =================================================================================================================
#
# List of registers, their purpose and their register type (which tells the resolution and units of the register)
#
# =================================================================================================================

# The register type is in brackets.

# EZ Config Registers

DesignCap_Reg = 0x18  # Expected capacity of the cell (Capacity) 
VEmpty_Reg = 0x3A  # The empty voltage threshold and the recovery voltage (Special, read docs)
ModelCfg_Reg = 0xDB  # Controls the basic options of the EZ algorithm (Special)
IchgTerm_Reg = 0x1E  # Detects when a charge cycle has been completed with a set of conditions (Current)
HibCfg_Reg = 0xBA  # The HibCfg register controls hibernate mode functionality
Command_Reg = 0x60  # Unless told explicitly to use this, do not use this. 
Config_Reg = 0x1D  # Just to set bit 15 for TSEL. 

# Algorithm Battery Parameters

FullSOCThr_Reg = 0x13  # Percentage of the capacity to consider as full voltage (Percentage)
RComp0_Reg = 0x38  # Characterisation information for calculating open-loop voltage (Special)
TempCo_Reg = 0x39  # Temperature compensation information (Special)

# Algorithm Output Registers

RepCap_Reg = 0x05  # Reported remaining capacity in mAh (Capacity) 
RepSOC_Reg = 0x06  # Reported state-of-charge percentage output for GUI (Percentage) 
FullCapRep_Reg = 0x10  # reports the full capacity that goes with RepCap, generally used for reporting to the GUI (Capacity)
FullCap_Reg = 0x35  # Don't use these, use FullCapRep instead. 
FullCapNom_Reg = 0x23
TTE_Reg = 0x11  # holds the estimated time to empty for the application under present temperature and load conditions (Time)
TTF_Reg = 0x20  # holds the estimated time to full for the application under present conditions (Time)
Cycles_Reg = 0x17  # holds a total count of the number of charge/discharge cycles of the cell that have occurred (Special)
Status_Reg = 0x00  # holds all flags related to alert thresholds and battery insertion or removal (Special)
Age_Reg = 0x07  # holds a calculated percentage value of the application’s present cell capacity compared to its original design capacity (Percentage)

# Analogue Measurements
# Voltage Measures

VCell_Reg = 0x09  # measures the per cell voltage (Voltage)
AvgVCell_Reg = 0x19  # measures the average of VCell readings (Voltage)
MinMaxVolt_Reg = 0x1B  # holds the maximum and minimum of VCell register values since device reset (Special)

# Current Measures

# We are using a 2mOhm sense resistor.

Current_Reg = 0x0A  # measures the current with a resolution of ± 25.6A (± 781.25μA) (Current)
AvgCurrent_Reg = 0x0B  # reports an average of Current register readings (Current)
MaxMinCurr_Reg = 0x1C  # holds the maximum and minimum Current register values since reset or until cleared by host software (Special)

# Temperature Measures

Temp_Reg = 0x08  # measures the temperature from the NTC external thermistor (set with Config.TSEL) (Temperature)
AvgTA_Reg = 0x16  # reports an average of the readings from the Temp register (Temperature)
MaxMinTemp_Reg = 0x1A  # maintains the maximum and minimum Temp register values since reset or until cleared by host software (Special)
DieTemp_Reg = 0x034  # measures the internal die temperature (Temperature)

# Power Measures

Power_Reg = 0xB1  # Instant power calculation from immediate current and voltage (8μV2) / Rsense)
AvgPower_Reg = 0xB3  # Filtered average power from the Power register (8μV2) / Rsense)

# Alert Functions

VAlrtTh_Reg = 0x01  # sets upper and lower limits that generate an alert if exceeded by the VCell register value (Special, read docs)
TAlrtTh_Reg = 0x02  # sets upper and lower limits that generate an alert if exceeded by the Temp register value (Special, read docs)
SAlrtTh_Reg = 0x03  # sets upper and lower limits that generate an alert if exceeded by RepSOC (Special, read docs)
IAlrtTh_Reg = 0xB4  # sets upper and lower limits that generate an alert if exceeded by the Current register value (Special, read docs)

# LED Config Registers

LEDCfg1_Reg = 0x40  # configures the LED driver operations, same for 2 and 3. Read the docs if you're interested. 
LEDCfg2_Reg = 0x4B
LEDCfg3_Reg = 0x37
CustLED_Reg = 0x64



class BatteryNode(Node):
    
    """
    Initialises the BatteryNode, default battery characteristic values stored in the battery_params dict in the class. 


    :return: BatteryNode object
    :rtype: BatteryNode
    """

    battery_params = {  # Read here and find the registers to find more detailed information: https://www.analog.com/media/en/technical-documentation/user-guides/max1726x-modelgauge-m5-ez-user-guide.pdf
        "DesignCap": 0x1388,  # The expected design capacity of the cell. (5000mAh)
        "IchgTerm": 0,  # Termination current, when charging current is between 0.125 * Ichgterm and 1.25 * Ichgterm it will terminate.
        "VEmpty": 0xA561, # VEmpty is considered 0%, the empty voltage target. Default is 3.3V, and a recovery voltage of 3.88V. 
        #For VEmpty, the docs are odd. It says a 10mV resolution with 8 bits, which is 256 levels(??), it probably is wrong at it is actually 20mV per level. 0-5.11V 
        "FullSOCThr": 0x5005 # when VFSOC > FullSOCThr, it will stop charging. Default is 95% (0x5F05), 80% is 0x5005.
        } 
    
    # Apparently 20%-80% is good for longetivity, research needs to be done on that though. 

    model_cfg = 0b1000010000000000  # refer to the ModelCFG page. Bit 10 and 15 are set (hopefully the correct endian)

    def __init__(self):  # initialising the ROS2 node
        super().__init__("battery_node")

        """
        Initialises timers, the subscriber and publisher for communication with the control node. Calls the _init_chip method to start the MAX17263. 
        """

        self.i2c_address = 0x02 # placeholder address 
        self.bus = smbus.SMBus("/dev/i2c-1")  # I believe that this is the /dev/ bus number. This is different
        # from the I2C address, which will be found once I hook the MAX17263 chip up with the Pi.
 
        timer_frequency = 1 # frequency of battery updates (Hz)
        self._timer = self.create_timer(1/timer_frequency, self.get_battery_state)  # for updating the battery status  
        self._register_timeout = self.create_rate(1e-3, self.get_clock())  # for timeouts for verifying registers
        self._timeout = self.create_rate(1e-2, self.get_clock())  # for timeouts for initialising the chip
        self._save_charge = self.create_timer(10, self._save_params)  # every 10s, check bit 6 of the Cycles register to save charge parameters. 
        self._check_IC_reset  = self.create_timer(30, self._init_chip) # every 30s, check if the fuel gauge has been reset, and re-init if it has. 

        self.battery_pub = self.create_publisher(msg_type=BatteryState, topic="/battery/output", qos_profile=QoS)
        self.battery_sub = self.create_subscription(msg_type=Int16MultiArray, topic="/battery/input", qos_profile=QoS, callback=self._battery_callback)

        self.state = {}
        self._init_chip()

    def _init_chip(self):  # initialising the IC chip on powerup.

        """Goes through the given startup commands to wake up the MAX17263 chip as described here:

        https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf
        """

        STATUS_POR_BIT = self._read_register(Status_Reg) & 0x0002

        if STATUS_POR_BIT == 0: # check if the IC has been reset.
            self.get_battery_state()
            return

        HibCFG = self._read_register(HibCfg_Reg)  # Store original HibCFG values
        self._write_register(Command_Reg, 0x90)  # exiting hibernate mode step 1
        self._write_register(HibCfg_Reg, 0x0)  # step 2
        self._write_register(Command_Reg, 0x0)  # step 3

        msg = f"Timeout: the IC MAX17263 did not clear the DNR bit in the Fstat register within 10 seconds."
        self._wait(0x3D, 0x01, msg)  # Wait for Fstat.DNR bit clear. 
        
        self._write_register(DesignCap_Reg, self.battery_params["DesignCap"])  # writing params to the chip. 
        self._write_register(IchgTerm_Reg, self.battery_params["IchgTerm"])
        self._write_register(VEmpty_Reg, self.battery_params["VEmpty"])
        self._write_and_verify_register(FullSOCThr_Reg, self.battery_params["FullSOCThr"])

        self._write_register(Config_Reg, self._read_register(Config_Reg) | 0x8000)  # Setting TSEL to 1 for external NTC. Might need to be placed elsewhere in case it breaks something.
        self._write_register(ModelCfg_Reg, self.model_cfg)  # Set the ModelCfg register
         
        msg_2 = f"Timeout: the ModelCFG.Refresh bit (bit 15) confirming model loading was not cleared."
        self._wait(ModelCfg_Reg, 0x8000, msg_2) # Poll ModelCFG.Refresh(highest bit) until it becomes 0 to confirm IC completes model loading.

        json_data = self._read_json()

        if json_data: # if the return value is not None, write to registers. 
            self._write_register(RComp0_Reg, self._hex_to_dec(json_data["RComp0"]))
            self._write_register(TempCo_Reg, self._hex_to_dec(json_data["TempCo"]))

         # THE DOCUMENTATION IS SHITE, WHAT THE FUCK IS A QRTABLE RAHHHHHH
        # Basically it doesn't explain at all what is a QRTable and which characterisation params to write
        # so I'll just ignore it for now. We need to figure out tf it means though. 

        # We aren't supposed to touch this??? Or something?? After looking up other ModelGauge chips
        # with this parameter apparently we're not supposed to use this but it asks to write it in the
        # guidance documentation? Help I'm losing it

        self._write_register(HibCfg_Reg, HibCFG) # restore original HibCFG values. 

    @staticmethod
    def _hex_to_dec(hex):
        """Converts hexidecimal strings into actual base-16 numerical values.

        :return: hexidecimal value
        :rtype: int
        """
        return int(hex, 16)

    def _write_json(self, json_data, file="battery_parameters.json"):
        """Write to the settings json file in the case there is an unexpected power outage.

        :param json_data: dictionary of setting values.
        :type json_data: dict
        :param file: file path, defaults to local directory "battery_parameters.json".
        :type file: str, Pathlike object
        """

        with open(file, "w") as fs:
            json.dump(json_data, fs)

    def _read_json(self, file="battery_parameters.json"):

        """Read the settings json file.

        :return: dictionary of setting values
        :rtype: dict
        """

        try:
            with open(file, "r") as fs:
                json_data = json.load(fs)
            return json_data
        
        except FileNotFoundError:
            self.get_logger().error(f"FileNotFoundError: {file} was not found, falling back to default values.")
            return None
        
    def _wait(self, register, bit_mask, error_msg, timeout_seconds=10):

        """ Waits for a bit or a set of bits to be toggled.

        :param register: register to wait on.
        :type register: int
        :param bit_mask: bit mask to filter for the concerned bits.
        :type bit_mask: int
        :param error_msg: error message if the timeout is exceeded.
        :type error_msg: str
        :param timeout_seconds: _(optional)_ timeout time, defaults to 10 seconds.
        :type timeout_seconds: int
        """

        _timeout_count = 0  # 1000 * 1e-2 = 10 seconds
        while self._read_register(register) & bit_mask != 0:
            self._timeout.sleep()
            _timeout_count += 1
            if _timeout_count * 1e-2 > timeout_seconds:
                self.get_logger().error(error_msg)
                break

    def _read_register(self, register):  # uint8 register value

        """Read a register.

        :param register: register to read.
        :type register: int
        :return: register's stored value.
        :rtype: int
        """

        register_value = self.bus.read_word_data(self.i2c_address, register)
        return register_value


    def _write_register(self, register, value):  # uint8 reg, uint16 value
        """Write to a register.

        :param register: register to be written to.
        :type register: int
        :param value: word of data to be written.
        :type value: int
        """

        self.bus.write_word_data(self.i2c_address, register, value)


    def _write_and_verify_register(self, register, value, attempts=3): # uint8 reg, uint16 value

        """Write to a register and verify that the value is written properly.

        :param register: register to be written to.
        :type register: int
        :param value: word of data to be written.
        :type value: int
        :param attempts: _(optional)_ number of attempts before giving up and sending an error, defaults to 3.
        :type attempts: int
        """

        _attempts = 0
        
        while True:
            self._write_register(register, value)
            self._register_timeout.sleep()
            if value != self._read_register(register):
                _attempts += 1
            elif _attempts >= attempts:
                self.get_logger().error(f"Write Error: failed to write data ({value}) to register {register}.")
                break
            else:
                break


    def _save_params(self): 

        """Save parameters to a json file every 64% that is charged and discharged.
        """

        if self._read_register(Cycles_Reg) & 0b100000:  # check bit 6. COULD BE WRONG IF THE ENDIAN IS WRONG. 
            SAVED_RCOMP0 = self._read_register(RComp0_Reg)  # characterisation information for open circuit operation
            SAVED_TempCo = self._read_register(TempCo_Reg)  # temperature compensation information for RComp0 reg
            SAVED_FullCapRep = self._read_register(FullCapRep_Reg) # reports the full capacity that goes with RepCap, generally used for reporting to the GUI
            SAVED_Cycles = self._read_register(Cycles_Reg)  # Total number of charge/discharge cycles of the cell that has occured. 
            # Cycles has a full range of 0 to 655.35 cycles with a 1% LSB. 
            SAVED_FullCapNom = self._read_register(FullCapNom_Reg)  # The full discharge capacity compensated according to present conditions. 

        json_data = {
            "RComp0": hex(SAVED_RCOMP0),
            "TempCo": hex(SAVED_TempCo),
            "FullCapRep": hex(SAVED_FullCapRep),
            "Cycles": hex(SAVED_Cycles),
            "FullCapNom": hex(SAVED_FullCapNom)
                     }
        
        self._write_json(json_data)


    def _battery_callback(self, msg):

        """Callback function from the subscriber, calls the get_battery_state method. 
        Sets the BatteryState values and publishes to the publisher. 
        """

        self.get_battery_state()

        # https://learn.adafruit.com/scanning-i2c-addresses/raspberry-pi 
        # The I2C address can be found here

        # Some form of data processing here to interpret the data, then convert it to a stdmsg type to send over ROS. 

        msg = BatteryState() # Create a message of this type, parameters are here: https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html
        msg.voltage = self._conv(self.state["AvgVCell"], "Voltage")  # set this to the battery voltage
        msg.percentage = self._conv(self.state["RepSOC"], "Percentage")  # # charge percentage normalised from 0 to 1. 
        msg.current = self._conv(self.state["AvgCurrent"], "Current") # discharge current, negative when discharging. 
        msg.charge = self._conv(self.state["RepCap"], "Capacity") # charge in Ah.
        msg.capacity = self._conv(self.state["FullCapRep"], "Capacity")  # capacity in Ah. 
        msg.design_charge = self._conv(self.state["DesignCap"], "Capacity")  # Design capacity

        # Power supply status:
        # 0 = UNKNOWN
        # 1 = CHARGING
        # 2 = DISCHARGING
        # 3 = NOT_CHARGING
        # 4 = FULL

        # msg.power_supply_status = 0 # haven't found an easy way to check the exact state of the battery with a single register.

        msg.battery_present = self.state["BatteryPresent"]

        self.battery_pub.publish(msg) # Publish BatteryState message 


    def _conv(self, value, register_type):

        """Converts the register value with the documentation resolutions for each type.
        
        :param value: register's value
        :type value: int
        :param register_type: register's type, refer to the list of aliases at the start of the file.
        :type register: str
        :return: converted value
        :rtype: float, None
        """

        level = int.from_bytes(value)
        match register_type:
            case "Capacity":
                # SENSE resistor is 2mOhms, hence we have a resolution of 2.5mAh/level. 
                return round(2.5e-3 * level)
            
            case "Percentage":    
                # 256 levels, 1/256% per level.
                return round(100 * (level / 256), 1)
            
            case "Voltage":
                # 78.125uV/level, with a maximum of 5.11992V (per cell basis)
                return round(78.125e-6 * level, 2)

            case "Current":
                # 0.78125mA/level, minimum of -25.6A and maximum of 25.5992A
                return round(0.78125e-3 * level)

            case "Temperature":
                # (1/256)C/level, minimum of -128.0 and maximum of 127.996. 
                return round((1/256) * level, 1)
            
            case "Resistance":
                # 1/4096 per level, minimum of 0 and maximum of 15.99976.
                return round((1/4096) * level, 1)

            case "Time":
                # 5.625s per level, minimum of 0 and maximum of 102.3984hrs. 
                return (5.625 * level)

            case "Special":
                self.get_logger().info("Should implement manually.")
                

    def get_battery_state(self):

        """Reads the register values and stores it in a dictionary in the BatteryNode object. 
        """

        self.state["RepCap"] = self._read_register(RepCap_Reg)  # Capacity in mAh  
        self.state["RepSOC"] = self._read_register(RepSOC_Reg)  # Capacity as a percentage, 256 levels. Round to nearest int. 
        # SOC means State of Charge. 
        self.state["TTE"] = self._read_register(TTE_Reg)  # Time to Empty value, each level is 5.25 seconds. 
        self.state["AvgVCell"] = self._read_register(AvgVCell_Reg) # NOT SURE IF TO USE VFSOC/VFOCV vs AvgVCell. 
        # The AvgVCell is an analog reading before being processed by the modelGauge algorith. 
        
        self.state["AvgCurrent"] = self._read_register(AvgCurrent_Reg) # Sustained peak current in mA 
        self.state["FullCapRep"] = self._read_register(FullCapRep_Reg)  # Capacity in Ah
        self.state["DesignCap"] = self._read_register(DesignCap_Reg)  # Design capacity in Ah
        # Age Register (%) = 100% x (FullCapRep Register / DesignCap Register)

        self.state["BatteryPresent"] = bool(self._read_register(Status_Reg) & 0b1000) # bit 3 of the Status Register is the Bst bit. 
        
        if self._read_register(Status_Reg) & 0x8000:  # bit 15 is the Battery removal bit, Br.
            for i in range(3):  # To hammer the point home, let's repeat this three times. 
                self.get_logger().error("Critical Error: Battery has been removed!") 

def main(args=None):
    rclpy.init(args=args)
    _battery_node = BatteryNode()
    rclpy.spin(_battery_node)
    _battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
