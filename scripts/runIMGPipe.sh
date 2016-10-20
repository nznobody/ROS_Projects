unset GTK_IM_MODULE
# Move to and source our ROS catkin workspace
cd ~/rex_ws
source devel/setup.bash

roslaunch rex_gridmap_buffer default.launch &
sleep 1
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 1
roslaunch rex_interface default.launch &
sleep 1
roslaunch grid_map_demos imgPipe.launch &
sleep 1
roslaunch rex_traversibility imgPipe.launch &
sleep 1
#rostopic pub /none_pose geometry_msgs/PoseWithCovarianceStamped '{header: {stamp: now, frame_id: "map"}, pose: {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}' &

rosrun rviz rviz -d ~/.rviz/img.rviz &
echo "Done"
