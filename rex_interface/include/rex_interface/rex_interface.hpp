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
#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <rex_interface/stepQuery.h>
#include <traversability_msgs/CheckFootprintPath.h>
#include <traversability_msgs/FootprintPath.h>
#include <traversability_msgs/TraversabilityResult.h>

class	RexInterface
{
public:
	RexInterface(ros::NodeHandle& nodeHandle);
	~RexInterface();
	
	/*!
   * Attempts to take a step forward form current position.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
	bool stepForward(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	
	/*!
   * Attempts to take a step in the given direction.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
	bool step( rex_interface::stepQuery::Request& request, rex_interface::stepQuery::Response& response );
	
	/*!
   * Reads in all the parameters from the parameter server.
   * @return true if successful.
   */
	bool readParamters();
	
	/*!
   * Callback for a movement goal message. Well check if footstep is possible at goal.
   * @return true if successful.
   */
	void stepQueryCallback(const geometry_msgs::PoseStampedConstPtr&	message );
	
private:
	ros::NodeHandle& nodeHandle_;
	ros::Publisher markerPublisher_;
	ros::Subscriber	stepQueryPoseSubscriber_;
	ros::ServiceServer stepForwardService_;
	ros::ServiceServer stepService_;
	ros::ServiceClient footprintCheckerSubscriber_;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	
	//parameters
	std::string footprintServiceName_;
	std::string footprintFrame_;
	double		stepForwardDistance_;
	double		stepBackwardDistance_;
	double		stepSidewaysDistance_;
	double		footprintRadius_;
	double		safeTraverse_;
	
	/// <summary>Joystick sectors</summary>
	enum EJoystickSector : int
	{
		NO_POSITION = 0,
		INSIDE_N,
		INSIDE_NE,
		INSIDE_E,
		INSIDE_SE,
		INSIDE_S,
		INSIDE_SW,
		INSIDE_W,
		INSIDE_NW,
		OUTSIDE_N,
		OUTSIDE_NNE,
		OUTSIDE_NE,
		OUTSIDE_ENE,
		OUTSIDE_E,
		OUTSIDE_ESE,
		OUTSIDE_SE,
		OUTSIDE_SSE,
		OUTSIDE_S,
		OUTSIDE_SSW,
		OUTSIDE_SW,
		OUTSIDE_WSW,
		OUTSIDE_W,
		OUTSIDE_WNW,
		OUTSIDE_NW,
		OUTSIDE_NNW,
	};
	
	enum ResultCode : int
	{
		OK = 0,
		NOT_TRAVERSABLE,
		ERROR = -1,
	};
};