import gpiozero as gpio

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


class DRV8701_Motor_LGPIO(Motor):
    def __init__(self, forward, backward, *, enable=None, pwm=True, pin_factory=None, pwm_frequency=None):
        super().__init__(forward, backward, enable=enable, pwm=pwm, pin_factory=pin_factory)

        if not pwm_frequency:
            raise ValueError(f"Please set a PWM frequency for this motor!")
        
        self.forward_device.frequency = pwm_frequency 
        self.backward_device.frequency  = pwm_frequency

    def stop(self):
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

