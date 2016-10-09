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
#include <mutex>
#include <pcl/common/common_headers.h>
#include <Eigen/Core>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
 
#include "../include/rex_gridmap_buffer/rex_gridmap_buffer.hpp"

//Thread locking mutex
std::mutex mtx_map_;
grid_map::GridMap	map_;

//Declerations
void subCallback(const grid_map_msgs::GridMapConstPtr&	msg);
bool mapSrvCallback(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
	ROS_INFO("Closing gridmap buffer node");
  // All the default sigint handler does is call shutdown()
	ros::shutdown();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rex_gridmap_buffer", ros::init_options::NoSigintHandler);
	
	//Initialise paramters
	std::string input_topic = "/rex_gridmapping/elevation_map_raw";
	//ToDo: Add PCD -> gridmap function!
	
	ros::NodeHandle n;
	ros::NodeHandle nh_ns("~");	//Local Namespace NodeHandle
	
	//Read parameters from server or launch file
	nh_ns.getParam("input_topic", input_topic);
	
	//Advertise service to get map
	ros::ServiceServer service = nh_ns.advertiseService("getSubMap", mapSrvCallback);
	
	//Subscribe to gridmapping topic
	ros::Subscriber sub = n.subscribe(input_topic, 2, subCallback);
	
	ROS_INFO("Starting gridmap buffer node");
	
	ros::spin();
	
	return 0;
}

void subCallback(const grid_map_msgs::GridMapConstPtr&	msg )
{
	ROS_INFO("New Map received");
	mtx_map_.lock();
	grid_map::GridMapRosConverter::fromMessage(*msg, map_);
	mtx_map_.unlock();
}

bool mapSrvCallback(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	bool isSuccess = true;
	ROS_INFO("Service requested a map");
	mtx_map_.lock();
	grid_map::GridMapRosConverter::toMessage(map_, response.map);
	mtx_map_.unlock();
	return isSuccess;
}