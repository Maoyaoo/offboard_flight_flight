#!/bin/bash
source ~/.bashrc
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4 mavros_posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun offboard_code offb_takeoff_node; exec bash"' \
