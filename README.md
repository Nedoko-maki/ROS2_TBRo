## ROS2 TBRo Packages

# Includes: 
- COSMO
- AI node for TBRo Student-Led team

# To use:
Just add to your src and build using colcon in your parent directory, for example if you have a parent directory ros2_ws with a src directory inside, ```colcon build``` in the ros2_ws:

```colcon build --packages-select cosmo```

# Important:
Run the start_cosmo script with ```./start_cosmo.sh``` and select 1, 2 or exit. 

**This is using a venv with the relevant packages installed** (and ```--system-site-packages``` when setting up the venv to keep global packages like ros2 installed with the new environment.)

When you want to install new packages, run ```. ./venv/bin/activate``` in the root directory and use pip. Otherwise just use apt install (might break still, not too sure). To leave the venv, use ```deactivate```.



# Risk Changelog:

- Added Raspberry Pi debian source list to apt-get (!!!). Be careful what packages you install from there as its not officially supported by Ubuntu and it may break things in the OS. I did this to get raspi-config installed on the Pi. 


# TODO:

- Add perms for hardware PWM access. (pwmchip0, 1) (https://github.com/dotnet/iot/blob/main/Documentation/raspi-pwm.md)
- test and implement hardware PWM, we have two options:
    - 1. Ignore the problem and just use the given lib for hardware pwm python bindings, but then I have to write the equivalent motor handling code
    - 2. Use the Pin library in gpiozero's library to make my own version so I can pass that into the Motor function so I don't have to rewrite the motor handling code

