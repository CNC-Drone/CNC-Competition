#!/bin/bash

# 编译camera_pub
catkin_make -DCATKIN_WHITELIST_PACKAGES="camera_pub"

# 编译quadrotor_msgs
catkin_make -DCATKIN_WHITELIST_PACKAGES="quadrotor_msgs"

# 编译commu_bridge
catkin_make -DCATKIN_WHITELIST_PACKAGES="commu_bridge"

# 编译my_mavros
catkin_make -DCATKIN_WHITELIST_PACKAGES="my_mavros"

# 编译object_identify
catkin_make -DCATKIN_WHITELIST_PACKAGES="object_identify"

# 编译offb
catkin_make -DCATKIN_WHITELIST_PACKAGES="offb"

# 编译smart_drone
catkin_make -DCATKIN_WHITELIST_PACKAGES="smart_drone"

# 编译stereo
catkin_make -DCATKIN_WHITELIST_PACKAGES="stereo"

# 编译VINS-Fusion
catkin_make -DCATKIN_WHITELIST_PACKAGES="VINS-Fusion"





