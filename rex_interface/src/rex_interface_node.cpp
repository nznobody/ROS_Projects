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
#include <ros/ros.h>
 
#include "../include/rex_interface/rex_interface.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rex_interface");
	ros::NodeHandle nodeHandle("~");
	RexInterface	rexInterface(nodeHandle);
	
	std::cout << "Init done, spinning.\n";
	
	// Spin
	ros::AsyncSpinner spinner(0); // Use n threads
	spinner.start();
	ros::waitForShutdown();
	return 0;
}