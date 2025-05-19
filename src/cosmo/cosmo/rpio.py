import gpiozero as gpio
import smbus3 as smbus
import os, json
from pathlib import Path
# This is meant to be a container for all the IO/pins that can be accessed by any relevant module. 
#
# Import rpio and call either set_pin with the pin number, pin base type (gpiozero docs), and the kwargs for the base type.
# Uses gpiozero classes so look at the documentation there for the parameters and methods! 
#
# https://gpiozero.readthedocs.io/en/latest/


pins = {x: None for x in range(40)}  # https://gpiozero.readthedocs.io/en/latest/recipes.html#pin-numbering

InputPin = gpio.DigitalInputDevice  # shorter aliases for convenience
OutputPin = gpio.DigitalOutputDevice 
OutputPWMPin = gpio.PWMOutputDevice
Motor = gpio.Motor

DRV_CONTROL_INPUTS_HELP="""
DRV8701P (PWM) Control Interface   

nSLEEP  IN1 IN2 Description
0       X   X   Sleep mode; HBridge disabled    
1       0   0   Coast  
1       0   1   Reverse
1       1   0   Forward
1       1   1   Brake
"""


BATTERY_I2C_ADDRESS = 0x6C # address defined in the MAX17263 user guide. 'Look up slave address'. Could alternatively
        # be 0x36 for '7 MSb addresses', if 0x6C fails.
ADC_I2C_ADDRESS = 0x48  # check i2cdetect if this is correct or I got the wrong endian. 
BUS = smbus.SMBus("/dev/i2c-1")  # the /dev/ bus number. Make sure the freq isn't faster than 400kHz.
BATTERY_LOGGER = None  # Will be set when BatteryNode initialises. 
MOTOR_LOGGER = None # Will be set when MotorDriverNode initialises. 
# class DRV8701_Motor:

#     def __init__(self, 
#                  forward_pin: HardwarePWM, 
#                  backward_pin: HardwarePWM,
#                  pwm_frequency: float = 100
#                  ):
        
#         self.forward_pin = forward_pin
#         self.backward_pin = backward_pin
        
#         self.forward_pin.start(0)
#         self.backward_pin.start(0)

#         self.forward_pin.change_frequency(pwm_frequency)
#         self.backward_pin.change_frequency(pwm_frequency)

#     @property
#     def value(self):
#         """
#         Represents the speed of the motor as a floating point value between -1
#         (full speed backward) and 1 (full speed forward), with 0 representing
#         stopped.
#         """
#         return (self.forward_pin._duty_cycle - self.backward_pin._duty_cycle) / 100

#     @value.setter
#     def value(self, value):
#         if not -1 <= value <= 1:
#             raise ValueError("Motor value must be between -1 and 1")
#         if value > 0:
#             try:
#                 self.forward(value)
#             except ValueError as e:
#                 raise ValueError(e)
#         elif value < 0:
#             try:
#                self.backward(-value)
#             except ValueError as e:
#                 raise ValueError(e)
#         else:
#             self.stop()

#     @property
#     def is_active(self):
#         """
#         Returns :data:`True` if the motor is currently running and
#         :data:`False` otherwise.
#         """
#         return self.value != 0

#     def forward(self, speed):
#         """
#         Drive the motor forwards.

#         :param float speed:
#             The speed at which the motor should turn. Can be any value between
#             0 (stopped) and 1 (maximum speed).
#         """
#         if not 0 <= speed <= 1:
#             raise ValueError('forward speed must be between 0 and 1')
        
#         self.backward_pin.change_duty_cycle(0)
#         self.forward_pin.change_duty_cycle(speed*100)

#     def backward(self, speed):
#         """
#         Drive the motor backwards.

#         :param float speed:
#             The speed at which the motor should turn. Can be any value between
#             0 (stopped) and 1 (maximum speed).
#         """
#         if not 0 <= speed <= 1:
#             raise ValueError('backward speed must be between 0 and 1')
        
#         self.forward_pin.change_duty_cycle(0)
#         self.backward_pin.change_duty_cycle(speed*100)

#     def reverse(self):
#         """
#         Reverse the current direction of the motor. If the motor is currently
#         idle this does nothing. Otherwise, the motor's direction will be
#         reversed at the current speed.
#         """
#         self.value = -self.value

#     def brake(self):
#         """
#         Stop/Brake the motor.
#         """
#         self.forward_pin.change_duty_cycle(100)  # Custom functions for the DRV8701 motor. 
#         self.backward_pin.change_duty_cycle(100)

#     def coast(self):
#         """
#         Let the motor coast.
#         """
#         self.forward_pin.change_duty_cycle(0)
#         self.backward_pin.change_duty_cycle(0)

#     def __del__(self):
#         """
#         Shutdown the pins when deallocating the object. 
#         """
#         self.forward_pin.stop()
#         self.backward_pin.stop()

# If lgpio gives an error about the GPIO being busy, try closing VSCode to release any active terminals. 
# Check with sudo gpioinfo to see if any pins are being occupied by a program that isn't meant to be using
# them.


class DRV8701_Motor_LGPIO(Motor):
    def __init__(self, forward, backward, *, enable=None, pwm=True, pin_factory=None, pwm_frequency=None):
        super().__init__(forward, backward, enable=enable, pwm=pwm, pin_factory=pin_factory)

        if not pwm_frequency:
            raise ValueError(f"Please set a PWM frequency for this motor!")
        
        self.forward_device.frequency = pwm_frequency 
        self.backward_device.frequency  = pwm_frequency

    def brake(self):
        """
        Stop/Brake the motor.
        """
        self.forward_device.on()  # Custom functions for the DRV8701 motor. 
        self.backward_device.on()

    def coast(self):
        """
        Let the motor coast.
        """
        self.forward_device.off()  
        self.backward_device.off()
    
    def __del__(self):
        self.forward_device.close()
        self.backward_device.close()


def set_pin(pin_number, pin_type, **kwargs):
    if pins[pin_number] is None:
        pins[pin_number] = pin_type(pin_number, **kwargs)
    return pins[pin_number]

def get_pin(pin_number):
    return pins[pin_number]

# PWMPin0 = HardwarePWM(pwm_channel=0, hz=0.1, chip=0)
# PWMPin1 = HardwarePWM(pwm_channel=1, hz=0.2, chip=0)
# PWMPin2 = HardwarePWM(pwm_channel=0, hz=0.3, chip=1)
# PWMPin3 = HardwarePWM(pwm_channel=1, hz=0.4, chip=1)

def adc_write_register(register, word, debug=False):
    if debug: 
            MOTOR_LOGGER.debug( f"Writing value {bin(word)} to ADC register {hex(register)}") 
    BUS.write_word_data(ADC_I2C_ADDRESS, register, word)

def adc_write_address_pointer(register, debug=False):
    if debug: 
            MOTOR_LOGGER.debug( f"Writing ADC address pointer {hex(register)}")
    BUS.write_byte(ADC_I2C_ADDRESS, register)

def adc_read_register(debug=False):
    
    msb = BUS.read_byte(ADC_I2C_ADDRESS)
    lsb = BUS.read_byte(ADC_I2C_ADDRESS)
    ret = msb >> 8 | lsb # shift the MSB 8 bits and add on the LSB. 
    if debug: 
            MOTOR_LOGGER.debug( f"Reading ADC conversion register, value={bin(ret)}")
    return ret

# ========================================================================
#                           BatteryNode IO below
# ========================================================================

def detect_i2c(component):
        """Checks if an i2c address exists.

        :return: bool
        :rtype: bool
        """

        try: 
            match component:
                case "battery": _ = read_register(0x00)
                case "adc": _ = adc_read_register()
        except OSError as e:
            LOGGER = BATTERY_LOGGER if component=="battery" else MOTOR_LOGGER
            LOGGER.error(f"{e}: likely the I2C address does not exist, check with cli command i2cdetect.")
            raise OSError

def read_register(register, debug=True) -> int:  # uint8 register value

        """Read a register.

        :param register: register to read.
        :type register: int
        :return: register's stored value.
        :rtype: int
        :param debug: enable debugging output, defaults to True.
        :type debug: bool
        """
        if debug: 
            BATTERY_LOGGER.debug( f"Reading BMS register {hex(register)}")

        register_value = BUS.read_word_data(BATTERY_I2C_ADDRESS, register)
        return register_value


def write_register(register, value, debug=True) -> None:  # uint8 reg, uint16 value
    """Write to a register.

    :param register: register to be written to.
    :type register: int
    :param value: word of data to be written.
    :type value: int
    :param debug: enable debugging output, defaults to True.
    :type debug: bool
    """

    if debug: 
        BATTERY_LOGGER.debug( f"Writing value {hex(value)} to BMS register {hex(register)}")

    BUS.write_word_data(BATTERY_I2C_ADDRESS, register, value)


def write_and_verify_register(register, 
                              value, 
                              debug=True, 
                              attempts=3, 
                              sleep_rate=None) -> bool: # uint8 reg, uint16 value

    """Write to a register and verify that the value is written properly.

    :param register: register to be written to.
    :type register: int
    :param value: word of data to be written.
    :type value: int
    :param attempts: _(optional)_ number of attempts before giving up and sending an error, defaults to 3.
    :type attempts: int
    :param debug: enable debugging output, defaults to True.
    :type debug: bool
    :return: True if the value wrote properly, else False
    :rtype: bool
    """

    _attempts = 0
    
    while True:
        write_register(register, value, debug=debug)
        sleep_rate.sleep()
        if value != read_register(register, debug=debug):
            _attempts += 1
        elif _attempts >= attempts:
            BATTERY_LOGGER.error(f"Write Error: failed to write data '{hex(value)}' to BMS register {register}.")
            return False
        else:
            return True


def hex_to_dec(hex):
    """Converts hexidecimal strings into actual base-16 numerical values.

    :return: hexidecimal value
    :rtype: int
    """
    return int(hex, 16)

def write_json(json_data, file="./battery_data.json"):
        """Write to the settings json file in the case there is an unexpected power outage.

        :param json_data: dictionary of setting values.
        :type json_data: dict
        :param file: file path, defaults to local directory "./battery_data.json".
        :type file: str, Pathlike object
        """

        with open(relpath(file), "w") as fs:
            json.dump(json_data, fs)

def read_json(file="./battery_data.json"):

    """Read the settings json file.
    
    :param file: file path, defaults to local directory "./battery_data.json".
    :type file: str, Pathlike object
    :return: dictionary of setting values
    :rtype: dict
    """

    try:
        with open(relpath(file), "r") as fs:
            json_data = json.load(fs)
        
        if not json_data:
             raise ValueError
        
        return json_data
    
    except FileNotFoundError:
        BATTERY_LOGGER.warn(f"FileNotFoundError: {file} was not found, falling back to default values.")
    except ValueError:
        BATTERY_LOGGER.warn(f"ValueError: {file} was empty, falling back to default values.")

def relpath(filepath):
    # access filepaths relative to the current script. 
    # Use "./file/to/path" to access local directory files.
    return Path(os.path.dirname(__file__), filepath)

