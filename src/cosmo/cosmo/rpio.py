import gpiozero as gpio
import rpi_hardware_pwm as hw_pwm

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

# pwm0 = hw_pwm.HardwarePWM(pwm_channel=0, hz=0, chip=0)
# pwm1 = hw_pwm.HardwarePWM(pwm_channel=0, hz=0, chip=1)
# pwm0.start()

class DRV8701_Motor(Motor):
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