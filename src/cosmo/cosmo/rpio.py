import gpiozero as gpio
import smbus3 as smbus
import json
# This is meant to be a container for all the pins that can be accessed by any relevant module. 
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

"""
DRV8701P (PWM) Control Interface   

nSLEEP  IN1 IN2 Description
0       X   X   Sleep mode; HBridge disabled    
1       0   0   Coast  
1       0   1   Reverse
1       1   0   Forward
1       1   1   Brake
"""


I2C_ADDRESS = 0x6C # address defined in the MAX17263 user guide. 'Look up slave address'. Could alternatively
        # be 0x36 for '7 MSb addresses', if 0x6C fails.  
BUS = smbus.SMBus("/dev/i2c-1")  # the /dev/ bus number. Make sure the freq isn't faster than 400kHz.
LOGGER = None  # Will be set when BatteryNode initialises. 

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

# ========================================================================
#                           BatteryNode IO below
# ========================================================================

def read_register(register):  # uint8 register value

        """Read a register.

        :param register: register to read.
        :type register: int
        :return: register's stored value.
        :rtype: int
        """
        LOGGER.debug( f"Reading register {hex(register)}")
        register_value = BUS.read_word_data(I2C_ADDRESS, register)
        return register_value


def write_register(register, value):  # uint8 reg, uint16 value
    """Write to a register.

    :param register: register to be written to.
    :type register: int
    :param value: word of data to be written.
    :type value: int
    """
    LOGGER.debug( f"Writing value {hex(value)} to register {hex(register)}")
    BUS.write_word_data(I2C_ADDRESS, register, value)


    # def _write_and_verify_register(self, register, value, attempts=3): # uint8 reg, uint16 value

    #     """Write to a register and verify that the value is written properly.

    #     :param register: register to be written to.
    #     :type register: int
    #     :param value: word of data to be written.
    #     :type value: int
    #     :param attempts: _(optional)_ number of attempts before giving up and sending an error, defaults to 3.
    #     :type attempts: int
    #     """

    #     _attempts = 0
        
    #     while True:
    #         write_register(register, value)
    #         self._register_timeout.sleep()
    #         if value != read_register(register):
    #             _attempts += 1
    #         elif _attempts >= attempts:
    #             self.get_logger().error(f"Write Error: failed to write data '{hex(value)}' to register {register}.")
    #             break
    #         else:
    #             break


def hex_to_dec(hex):
    """Converts hexidecimal strings into actual base-16 numerical values.

    :return: hexidecimal value
    :rtype: int
    """
    return int(hex, 16)

def write_json(json_data, file="battery_parameters.json"):
        """Write to the settings json file in the case there is an unexpected power outage.

        :param json_data: dictionary of setting values.
        :type json_data: dict
        :param file: file path, defaults to local directory "battery_parameters.json".
        :type file: str, Pathlike object
        """

        with open(file, "w") as fs:
            json.dump(json_data, fs)

def read_json(file="battery_parameters.json"):

    """Read the settings json file.

    :return: dictionary of setting values
    :rtype: dict
    """

    try:
        with open(file, "r") as fs:
            json_data = json.load(fs)
        return json_data
    
    except FileNotFoundError:
        LOGGER.warn(f"FileNotFoundError: {file} was not found, falling back to default values.")
        return None

