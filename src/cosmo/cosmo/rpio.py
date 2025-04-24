import gpiozero as gpio


# This is meant to be a container for all the pins that can be accessed by any relevant module. 
#
# Import rpio and call either set_pin with the pin number, pin base type (gpiozero docs), and the kwargs for the base type.
# Uses gpiozero classes so look at the documentation there for the parameters and methods! 
#
# https://gpiozero.readthedocs.io/en/latest/


pins = {x: None for x in range(30)}

InputPin = gpio.DigitalInputDevice  # shorter aliases for convenience
OutputPin = gpio.DigitalOutputDevice 
OutputPWMPin = gpio.PWMOutputDevice


def set_pin(pin_number, pin_type, **kwargs):
    if pins[pin_number] is not None:
        pins[pin_number] = pin_type(pin_number, **kwargs)
    return pins[pin_number]

def get_pin(pin_number):
    return pins[pin_number]