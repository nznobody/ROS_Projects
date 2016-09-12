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
	
	//Load service handlers
	stepForwardService_ = nodeHandle_.advertiseService("step_forward", &RexInterface::stepForward, this);
}

RexInterface::~RexInterface()
{
	nodeHandle_.shutdown();
}

bool RexInterface::stepForward(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	
	return false;
}

bool RexInterface::readParamters()
{
	nodeHandle_.param("stepForwardDistance", stepForwardDistance_, 0.25);
	return false;
}
