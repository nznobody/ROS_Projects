unset GTK_IM_MODULE
# Move to and source our ROS catkin workspace
cd ~/rex_ws
source devel/setup.bash

source ~/scripts/coreAP.sh
sleep 5

roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 1
############################################################################
# Launch camera pipeline... time sensitive
roslaunch rex_zed_wrapper zed_ground_calib.launch &
sleep 2
#Then start the ground leveling calibration program
roslaunch rex_ground_calib default.launch &
sleep 5
#The ZED should be operational by now, calibrate to the ground angle, wait for completion
rosservice call /rex_ground_calib/trigger_calib
sleep 10
#Start the rest of the odometry pipeline:
roslaunch rex_odom2pose zedPipe.launch &
############################################################################
roslaunch rex_gridmapping zedRex.launch &
roslaunch rex_traversibility zedRex.launch &
sleep 1
roslaunch rex_interface rexPipe_safe.launch &
roslaunch rex_model load_model_xacro.launch &
#rosrun rviz rviz -d ~/.rviz/zedPipe.rviz &

source ~/scripts/throttle.sh

echo "Done"
