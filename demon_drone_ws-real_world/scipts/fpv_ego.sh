#export ROS_MASTER_URI=http://192.168.3.20:11311
#export ROS_HOSTNAME=192.168.3.20
#export ROS_IP=192.168.3.20

# source /home/khadas/demon_packages/ego_planner_ws/devel/setup.bash
source /home/khadas/demon_packages/demon_drone_ws/devel/setup.bash

gnome-terminal -t "mavros" --window -e 'bash -c "roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:921600" gcs_url:=udp-b://@; exec bash"' \
--tab -t "stereo"        -e 'bash -c "sleep 3; source /home/khadas/demon_packages/demon_drone_ws/devel/setup.bash; roslaunch stereo stereo.launch           ; exec bash"' \
--tab -t "vins"          -e 'bash -c "sleep 5; source /home/khadas/demon_packages/demon_drone_ws/devel/setup.bash; roslaunch vins stereo.launch             ; exec bash"' \
--tab -t "commu_bridge"  -e 'bash -c "sleep 7; source /home/khadas/demon_packages/demon_drone_ws/devel/setup.bash; roslaunch commu_bridge transfer.launch   > ~/log/transfer.out   ; exec bash"' \
--tab -t "Ego"           -e 'bash -c "sleep 9; source /home/khadas/demon_packages/ego_planner_ws/devel/setup.bash; roslaunch ego_planner ego.launch         > ~/log/ego.out         ; exec bash"' \
--tab -t "Ego_rviz"      -e 'bash -c "sleep 10; source /home/khadas/demon_packages/ego_planner_ws/devel/setup.bash;source /home/khadas/demon_packages/demon_drone_ws/devel/setup.bash; roslaunch ego_planner rviz.launch        ; exec bash"' \
--tab -t "htop"          -e 'bash -c "htop; exec bash"' \

# --tab -t "htop"          -e 'bash -c "sleep 4; rosrun mavros mavcmd long 511 31 3000 0 0 0 0 0 ; htop                                   ;    exec bash"' \

# gnome-terminal -t "keyboard" --window -e 'bash -c "sleep 20; source ~/Escort_Drone_ws/devel/setup.bash; roslaunch keyboard_ctr keyboard_ctr.launch; exec bash"' \
# --tab -t "gup_top"       -e 'bash -c "sleep 2; sudo intel_gpu_top;    exec bash"' \

# sleep 4
# rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0   # /mavros/imu/data
# rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0  # /mavros/imu/data_raw

# 录制rosbag ： rosbag record -O stereo_jy61p_imu /stereo_s/left/image_raw /stereo_s/right/image_raw /imu_node/data /mavros/imu/data
# 标定双目    ： kalibr_calibrate_cameras --target checkerboard.yaml --bag stereo_jy61p_imu.bag --models pinhole-radtan pinhole-radtan --topics /stereo_s/left/image_raw /stereo_s/right/image_raw
# 标定双目+imu: kalibr_calibrate_imu_camera --target checkerboard.yaml --cam camchain-stereo_jy61p_imu.yaml --imu imu.yaml --bag stereo_jy61p_imu.bag
