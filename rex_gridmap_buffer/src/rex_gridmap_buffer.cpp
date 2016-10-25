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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
 
#include "../include/rex_gridmap_buffer/rex_gridmap_buffer.hpp"

//Thread locking mutex
std::mutex mtx_map_;
grid_map::GridMap	map_;

int counter = 0;

//Declerations
void subCallback(const grid_map_msgs::GridMapConstPtr&	msg);
bool mapSrvCallback(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
bool savePCDCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

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
	nh_ns.getParam("input_topic",input_topic);
	
	//Advertise service to get map
	ros::ServiceServer service = nh_ns.advertiseService("getSubMap", mapSrvCallback);
	
	//Advertise service to get map
	ros::ServiceServer service2 = nh_ns.advertiseService("savePCD", savePCDCallback);
	
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
	ROS_INFO("Service requested a map from buffer node");
	
	//Only area of map, not whole map
	grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
	grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
	grid_map::Index index;
	
	mtx_map_.lock();
	grid_map::GridMap subMap = map_.getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
	mtx_map_.unlock();
	
	//Check what layers were requested...
	for (size_t i = 0; i < request.layers.size(); i++)
	{
		if (!subMap.exists(request.layers[i]))
		{
			subMap.add(request.layers[i]);
			subMap[request.layers[i]].setConstant(0.0);	//ensure values are all zero
		}
	}
	grid_map::GridMapRosConverter::toMessage(subMap, response.map);
	
	return isSuccess;
}

bool savePCDCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (map_.exists("elevation"))
	{
		sensor_msgs::PointCloud2 pointCloud;
		mtx_map_.lock();
		grid_map::GridMapRosConverter::toPointCloud(map_, "elevation", pointCloud);
		mtx_map_.unlock();
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(pointCloud, pcl_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
		std::string uniqueName = "/home/ubuntu/Documents/GridMap_Export" + std::to_string(counter) + ".pcd";
		pcl::io::savePCDFileASCII(uniqueName.c_str(), *temp_cloud);
		ROS_INFO("PCD Saved from GridMap Buffer");
		counter++;
	}
	return true;
}
