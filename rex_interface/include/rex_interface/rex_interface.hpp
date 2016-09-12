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
   * Reads in all the parameters from the parameter server.
   * @return true if successful.
   */
	bool readParamters();
	
private:
	ros::NodeHandle& nodeHandle_;
	ros::ServiceServer stepForwardService_;
	
	//parameters
	double	stepForwardDistance_;
	
};