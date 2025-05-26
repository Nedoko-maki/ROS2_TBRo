#! /usr/bin/bash

start_cosmo () {
printf "What would you like to do? \n1. Start the cosmo package.\n2. colcon build the cosmo package."
read -p ": " option 

if [ $option = 1 ]; then

if [ ! -d ./install ]; then
	echo "./install directory does not exist. colcon build the packages from the src directory to run."
	exit 2
fi

if [ ! -d ./launch ]; then
	echo "./launch directory does not exist. git clone again or download the cosmo launch file and put it in a new /launch/ dir."
	exit 2
fi

export RCUTILS_COLORIZED_OUTPUT=1
echo "Located /install and /launch."
source ./install/local_setup.bash
. ./venv/bin/activate
ros2 launch ./launch/cosmo_launch.py

elif [ $option = 2 ]; then

if [ ! -d ./src ]; then
	echo "./src directory does not exist. git clone again if you can."
	exit 2
fi

. ./venv/bin/activate
colcon build --packages-select cosmo cosmo_msgs
deactivate

elif [ $option = "exit" ]; then
	exit 1
else
printf "Not a valid choice! Try inputting 1 or 2, or exit if you would like to exit.\n"	
start_cosmo
fi 
}

start_cosmo