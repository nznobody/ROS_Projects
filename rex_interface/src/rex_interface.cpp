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
#include <pcl/common/common_headers.h>
#include <Eigen/Core>
 
#include "../include/rex_interface/rex_interface.hpp"



RexInterface::RexInterface(ros::NodeHandle& nodeHandle)
	: nodeHandle_(nodeHandle)
{
	readParamters();
	
	//Load publishers
	markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	
	//Load service handlers
	stepForwardService_ = nodeHandle_.advertiseService("step_forward", &RexInterface::stepForward, this);
	
	//Subscribe to services
	footprintCheckerSubscriber_ = nodeHandle_.serviceClient<traversability_msgs::CheckFootprintPath>(footprintServiceName_);
	
	//Subscribe to messages
	stepQueryPoseSubscriber_ = nodeHandle_.subscribe("/move_base_simple/goal", 1, &RexInterface::stepQueryCallback, this);
}

RexInterface::~RexInterface()
{
	nodeHandle_.shutdown();
}

bool RexInterface::stepForward(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	traversability_msgs::CheckFootprintPath	footprintService;
	traversability_msgs::FootprintPath	footprintPath;
	geometry_msgs::Pose					footprint;
	
	//Test code, should make a foot print at 0 0 0, facing in the x direction
	footprint.position.x = footprint.position.y = footprint.position.z = 0.0;
	//footprint.position.y = 0.0;
	footprint.orientation.x = 1.0;
	footprint.orientation.y = footprint.orientation.z = footprint.orientation.w = 0.0;
	
	//load footprints into service message
	footprintPath.poses.header.frame_id = footprintFrame_;
	footprintPath.poses.header.stamp = ros::Time::now();
	footprintPath.poses.poses.push_back(footprint);
	
	//assign radius, this could later be a polygon
	footprintPath.radius = footprintRadius_;
	
	//copy into service request
	footprintService.request.path.push_back(footprintPath);
	
	//call service
	footprintCheckerSubscriber_.waitForExistence(); //wait for it
	ROS_DEBUG("Calling footprint checker service");
	footprintCheckerSubscriber_.call(footprintService);
	
	if (footprintService.response.result.front().is_safe)
		ROS_INFO("Safe to step forward");
	else
		ROS_INFO("Not safe to step forward");
	
	//Testing Visualisation
	visualization_msgs::Marker marker;
	marker.header.frame_id = footprintFrame_;
	marker.header.stamp = ros::Time();
	marker.ns = "rex_interface";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = footprint.position.x;
	marker.pose.position.y = footprint.position.y;
	marker.pose.position.z = footprint.position.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = footprintRadius_*2;
	marker.scale.y = footprintRadius_*2;
	marker.scale.z = 0.1;
	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	markerPublisher_.publish(marker);
	
	double test = 0.0;
	test = footprintService.response.result.front().traversability;
	ROS_INFO("Traversibility: %f", footprintService.response.result.front().traversability);
	
	return true;
}

bool RexInterface::readParamters()
{
	nodeHandle_.param("footprintServiceName", footprintServiceName_, std::string("/rex_traversibility/check_footprint_path"));
	nodeHandle_.param("footprintFrame", footprintFrame_, std::string("map"));
	nodeHandle_.param("stepForwardDistance", stepForwardDistance_, 0.25);
	nodeHandle_.param("footprintRadius", footprintRadius_, 0.1);
	return false;
}

void RexInterface::stepQueryCallback(const geometry_msgs::PoseStampedConstPtr&	message)
{
	traversability_msgs::CheckFootprintPath	footprintService;
	traversability_msgs::FootprintPath	footprintPath;
	geometry_msgs::Pose					footprint;
	
	//Copy pose information into footprint message
	footprint.position.x = message->pose.position.x;
	footprint.position.y = message->pose.position.y;
	footprint.position.z = message->pose.position.z;
	
	footprint.orientation.x = message->pose.orientation.x;
	footprint.orientation.y = message->pose.orientation.y;
	footprint.orientation.z = message->pose.orientation.z;
	footprint.orientation.w = message->pose.orientation.w;
	
	//load footprints into service message
	footprintPath.poses.header.frame_id = footprintFrame_;	//This should be changed...
	footprintPath.poses.header.stamp = ros::Time::now();
	footprintPath.poses.poses.push_back(footprint);
	
	//assign radius, this could later be a polygon
	footprintPath.radius = footprintRadius_;
	
	//copy into service request
	footprintService.request.path.push_back(footprintPath);
	
	//call service
	footprintCheckerSubscriber_.waitForExistence(); //wait for it
	ROS_DEBUG("Calling footprint checker service");
	footprintCheckerSubscriber_.call(footprintService);
	
	//Testing Visualisation
	visualization_msgs::Marker marker;
	marker.header.frame_id = footprintFrame_;
	marker.header.stamp = ros::Time();
	marker.ns = "rex_interface";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = footprint.position.x;
	marker.pose.position.y = footprint.position.y;
	marker.pose.position.z = footprint.position.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = (footprintRadius_ * 2);
	marker.scale.y = (footprintRadius_ * 2);
	marker.scale.z = 0.1;
	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	markerPublisher_.publish(marker);
	
	double test = 0.0;
	test = footprintService.response.result.front().traversability;
	ROS_INFO("Traversibility: %f", footprintService.response.result.front().traversability);
	
}
