#!/bin/bash

source ~/.bashrc

gnome-terminal --window -- bash -c "roscore; exec bash" \
--tab -- bash -c "sleep 5; roslaunch px4 mavros_posix_sitl.launch; exec bash" \
--tab -- bash -c "sleep 10; rosrun offboard_code offb_takeoff_land_node; exec bash" 

