unset GTK_IM_MODULE
# Move to and source our ROS catkin workspace
cd ~/rex_ws
source devel/setup.bash

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
sleep 5
#Start the rest of the odometry pipeline:
roslaunch rex_odom2pose zedPipe.launch &
### Potentially remove / update these old sections
#roslaunch rex_zed_wrapper ekf.launch & #this runs the odometry filter
############################################################################
roslaunch rex_gridmapping zedPipe.launch &
roslaunch rex_traversibility zedPipe.launch &
sleep 1
roslaunch rex_interface default.launch &
rosrun rviz rviz -d ~/.rviz/zedPipe.rviz &
echo "Done"
