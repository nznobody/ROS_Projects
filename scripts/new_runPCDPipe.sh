unset GTK_IM_MODULE
# Move to and source our ROS catkin workspace
cd ~/rex_ws
source devel/setup.bash

rosrun pcl_ros pcd_to_pointcloud /home/ubuntu/samplePCD/REX_DemoStairs_Rex_Stairs_Rastered.pcd 5.0 &

roslaunch rex_gridmapping pcd.launch &

sleep 2

roslaunch rex_traversibility pcd.launch &
roslaunch rex_traversibility visualization.launch &

sleep 2

rostopic pub /none_pose geometry_msgs/PoseWithCovarianceStamped '{header: {stamp: now, frame_id: "map"}, pose: {pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}' &

rosrun rviz rviz -d ~/.rviz/pcd.rviz &

echo "Done"
