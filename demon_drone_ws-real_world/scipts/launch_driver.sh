#!/bin/bash
gnome-terminal --tab -- bash -c "roslaunch mavros px4.launch; exec bash"
sleep 1s  
gnome-terminal --tab -- bash -c "roslaunch stereo stereo.launch; exec bash"
sleep 1s  
gnome-terminal --tab -- bash -c "roslaunch offb run_ctrl.launch; exec bash"

