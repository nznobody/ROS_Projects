unset GTK_IM_MODULE
# Move to and source our ROS catkin workspace
cd ~/rex_ws
source devel/setup.bash

#First, start up the ZED Camera, and allow time for the GPU memory allocations
roslaunch rex_zed_wrapper zed_ground_calib.launch &
sleep 2

#Then start the ground leveling calibration program
roslaunch rex_ground_calib default.launch &
sleep 10

#The ZED should be operational by now, calibrate to the ground angle, wait for completion
rosservice call /rex_ground_calib/trigger_calib
sleep 5

#Start the rest of the pipeline: odometry filter, odom message converter, gridmapping and grid_visualiser
roslaunch rex_odom2pose default.launch &
roslaunch rex_gridmapping zed.launch &

roslaunch traversability_estimation rex.launch &
roslaunch traversability_estimation visualization.launch &

rosrun rviz rviz -d ~/.rviz/rex_mapping.rviz &

### Potentially remove / update these old sections
cd ~/catkin_ws
source devel/setup.bash
roslaunch zed_wrapper ekf.launch & #this runs the odometry filter

echo "Done"
