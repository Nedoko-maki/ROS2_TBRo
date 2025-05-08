## ROS2 TBRo Packages

# Includes: 
- COSMO for the GDBP 
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

- Add debugging colour console output
- Dockerise the whole thing?
