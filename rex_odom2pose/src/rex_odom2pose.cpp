//	========================================================================
//					Rex ROS Sample Core Node
//				Copyright(C) {2016}  {Manu Lange}
//
//	This program is free software : you can redistribute it and / or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program.If not, see < http ://www.gnu.org/licenses/>.
//	========================================================================
 

#include <iostream>
#include <ostream>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

void subCallback(const nav_msgs::Odometry::ConstPtr& msg);

ros::Publisher pubPose;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
	ros::shutdown();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "odom2pose", ros::init_options::NoSigintHandler);

	//std::cout << "Hello Manu" << std::endl;
	
	std::string odom_topic = "/odometry/filtered";
	std::string pose_topic = "/pose/filtered";
	
	ros::NodeHandle n;
	ros::NodeHandle nh_ns("~");	//Local Namespace NodeHandle
	nh_ns.getParam("input_topic", odom_topic);
	nh_ns.getParam("output_topic", pose_topic);
	
	ros::Subscriber sub = n.subscribe(odom_topic, 100, subCallback);
	
	pubPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 100);
	
	signal(SIGINT, mySigintHandler);
	
	ros::spin();
	
	ROS_WARN("Shutdown request received by odom2pose");
	sub.shutdown();
	pubPose.shutdown();
	ros::shutdown();
	return 0;
}

void subCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if (pubPose.getNumSubscribers())
	{
		geometry_msgs::PoseWithCovarianceStampedPtr	output(new geometry_msgs::PoseWithCovarianceStamped());	
		output->header = msg->header; //copy the message data
		output->pose = msg->pose;
		pubPose.publish(output); //Publish it
	}
}