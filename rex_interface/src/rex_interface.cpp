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
	: nodeHandle_(nodeHandle),
	tfListener(tfBuffer)
{
	readParamters();
	
	//Load publishers
	markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	
	//Load service handlers
	stepForwardService_ = nodeHandle_.advertiseService("step_forward", &RexInterface::stepForward, this);
	stepService_ = nodeHandle_.advertiseService("step", &RexInterface::step, this);
	
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
	visualise(footprint);
	
	double test = 0.0;
	test = footprintService.response.result.front().traversability;
	ROS_INFO("Traversibility: %f", footprintService.response.result.front().traversability);
	
	return true;
}

bool RexInterface::step(rex_interface::stepQuery::Request& request, rex_interface::stepQuery::Response& response)
{
	//Establish local variables
	traversability_msgs::CheckFootprintPath	footprintService;
	traversability_msgs::FootprintPath	footprintPath;
	geometry_msgs::Pose					footprint;
	response.resultCode = ResultCode::ERROR;	//default to error
	
	geometry_msgs::TransformStamped transformStamped;
	//Get transform to map
	try {
		transformStamped = tfBuffer.lookupTransform("map",
			"base_link",
			ros::Time(0),ros::Duration(1.0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
		return false;
	}
	
	//tf2::Vector3	footPosition;
	Eigen::Vector3d	footPosition;
	
	//Check if the direction queried is handled. This should become generic eventually.
	footPosition.z() = 0.0;	//Default z height
	footprint.orientation.x = footprint.orientation.y = footprint.orientation.z = footprint.orientation.w = 0.0;	//Default orientation
	switch (request.direction)
	{
	case OUTSIDE_N:
		footPosition.x() = stepForwardDistance_;
		footPosition.y() = 0.0;
		break;
	case OUTSIDE_E:
		footPosition.x() = 0.0;
		footPosition.y() =  -stepSidewaysDistance_;
		break;
	case OUTSIDE_S:
		footPosition.x() = -stepBackwardDistance_;
		footPosition.y() = 0.0;
		break;
	case OUTSIDE_W:
		footPosition.x() = 0.0;
		footPosition.y() = stepSidewaysDistance_;
		break;
	default:
		return false;	//Unhandled case, return an error.
	}
	
	//Transform frames
	Eigen::Affine3d	eigenTF = tf2::transformToEigen(transformStamped);
	footPosition = eigenTF*footPosition;
	//tf2::doTransform(footPosition, footPosition, transformStamped);

	//load footprints into service message
	footprint.position.x = footPosition.x();
	footprint.position.y = footPosition.y();
	footprint.position.z = footPosition.z();
	footprintPath.poses.header.frame_id = footprintFrame_;
	footprintPath.poses.header.stamp = ros::Time::now();
	footprintPath.poses.poses.push_back(footprint);
	
	//Using polygon
	footprintPath.radius = 0.0;	//Testing polygons
	footprintPath.footprint.header = footprintPath.poses.header;

	for (size_t i = 0; i < footprintPoints_.size(); i++)
	{
		footprintPath.footprint.polygon.points.push_back(footprintPoints_[i]);
	}

	//copy into service request
	footprintService.request.path.push_back(footprintPath);

	//call service
	footprintCheckerSubscriber_.waitForExistence(); //wait for it
	ROS_DEBUG("Calling footprint checker service");
	footprintCheckerSubscriber_.call(footprintService);

	//Testing Visualisation
	visualise(footprint);

	double result = 0.0;
	result = footprintService.response.result.front().traversability;
	ROS_INFO("Traversibility: %f", result);
	
	if (result >= safeTraverse_)	//Todo: Figure out why sometimes it returns values around 0.95 when some cells are 'unseen'
		response.resultCode = ResultCode::OK;
	else
		response.resultCode = ResultCode::NOT_TRAVERSABLE;
	
	//Todo: Figure out how to handle unmapped terrain properly...? Maybe the isSafe flag on footpath response?

	return true;
}

bool RexInterface::readParamters()
{
	//Added footprint for query
	// Read footprint polygon.
	XmlRpc::XmlRpcValue footprint;
	if (nodeHandle_.getParam("footprint/footprint_polygon", footprint)) {
		if (footprint.size() < 3) {
			ROS_WARN("Footprint polygon must consist of at least 3 points. Only %i points found.", footprint.size());
			footprintPoints_.clear();
		}
		else {
			geometry_msgs::Point32 pt;
			pt.z = 0.0;
			for (int i = 0; i < footprint.size(); i++) {
				pt.x = (double) footprint[i][0];
				pt.y = (double) footprint[i][1];
				footprintPoints_.push_back(pt);
			}
		}
	}
	else {
		ROS_WARN("Traversability Map: No footprint polygon defined.");
	}
	
	nodeHandle_.param("footprintServiceName", footprintServiceName_, std::string("/rex_traversibility/check_footprint_path"));
	nodeHandle_.param("footprintFrame", footprintFrame_, std::string("base_link"));
	nodeHandle_.param("stepForwardDistance", stepForwardDistance_, 0.40);
	nodeHandle_.param("stepBackwardDistance", stepBackwardDistance_, 0.25);
	nodeHandle_.param("stepSidwaysDistance", stepSidewaysDistance_, 0.10);
	nodeHandle_.param("footprintRadius", footprintRadius_, 0.15);
	nodeHandle_.param("safeTraverse", safeTraverse_, 0.80);
	return true;
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
	footprintPath.radius = 0.0;	//Testing polygons
	footprintPath.footprint.header = footprintPath.poses.header;

	for (size_t i = 0; i < footprintPoints_.size(); i++)
	{
		footprintPath.footprint.polygon.points.push_back(footprintPoints_[i]);
	}
	
	//copy into service request
	footprintService.request.path.push_back(footprintPath);
	
	//call service
	footprintCheckerSubscriber_.waitForExistence(); //wait for it
	ROS_DEBUG("Calling footprint checker service");
	footprintCheckerSubscriber_.call(footprintService);
	
	//Testing Visualisation
	visualise(footprint);
	
	double test = 0.0;
	test = footprintService.response.result.front().traversability;
	ROS_INFO("Traversibility: %f", footprintService.response.result.front().traversability);
	
}


void RexInterface::visualise(const geometry_msgs::Pose footprint)
{
	//Testing Visualisation
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "rex_interface";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = footprint.position.x;
	marker.pose.position.y = footprint.position.y;
	marker.pose.position.z = footprint.position.z;
	marker.pose.orientation.x = footprint.orientation.x;
	marker.pose.orientation.y = footprint.orientation.y;
	marker.pose.orientation.z = footprint.orientation.z;
	marker.pose.orientation.w = footprint.orientation.w;
	
	for (size_t i = 0; i < footprintPoints_.size(); i++)
	{
		geometry_msgs::Point test2;
		test2.x = footprintPoints_[i].x;
		test2.y = footprintPoints_[i].y;
		test2.z = footprintPoints_[i].z;
		marker.points.push_back(test2);
	}
	marker.scale.x = 0.05;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 0.75; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	markerPublisher_.publish(marker);
}
