unset GTK_IM_MODULE
# Move to and source our ROS catkin workspace
cd ~/rex_ws
source devel/setup.bash

roslaunch rex_gridmap_buffer bag.launch &
sleep 1
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 1
roslaunch rex_interface default.launch &
rosrun rviz rviz -d ~/.rviz/zedPipe.rviz &
sleep 2
roslaunch rex_traversibility bagPipe.launch &
echo "Done"
