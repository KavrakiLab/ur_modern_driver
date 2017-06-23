#!/bin/bash
## A script that intializes the ach and sns setup needed to run on a single ur.

### Vars
ACH_STATE="state"
ACH_BLEND_REF="ref"
ACH_WD_REF="wd-ref"
ACH_FOLLOW_PATH="follow_path"
ACH_FINISHED_PATH="path_finished"

## ENV VARS
source /home/kavrakilab/ur5_wksp/install/setup.bash
source /home/kavrakilab/ur5_wksp/devel/setup.bash
export SNS_SCENE_PLUGIN=libkavrakilab-one-arm-scene.so
export SNS_SCENE_NAME=kavrakilab

### Init
sns start
ach mk -1 $ACH_STATE
ach mk -1 $ACH_BLEND_REF
ach mk -1 $ACH_WD_REF
ach mk -1 $ACH_FOLLOW_PATH
ach mk -1 $ACH_FINISHED_PATH

### Spin-ups
roscore &
sleep 1
rosrun ur_modern_driver ur_realtime_driver -y $ACH_STATE \
	-u $ACH_BLEND_REF -p 1\
        -u $ACH_WD_REF -p 10\
	-r 192.168.0.20 &
sleep 1 # wait for it to start
sns-pblend -y $ACH_STATE -u $ACH_BLEND_REF -w $ACH_FOLLOW_PATH -f $ACH_FINISHED_PATH &
sleep 1
sns-watchdog -y $ACH_STATE -j $ACH_BLEND_REF -u $ACH_WD_REF &
sleep 1
rosrun ur_modern_driver ros_realtime_interface &

## Now you're ready to send waypoint through ROS to /ur_follow_trajectory!
