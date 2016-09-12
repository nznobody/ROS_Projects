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
 
#include "../include/rex_synthetic/rex_synthetic.hpp"

namespace grid_map_demos {

	ImageToGridmapDemo::ImageToGridmapDemo(ros::NodeHandle& nodeHandle)
		: nodeHandle_(nodeHandle)
		, map_(grid_map::GridMap({ "elevation" }))
		, mapInitialized_(false)
	{
		readParameters();
		map_.setBasicLayers({ "elevation" });
		imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmapDemo::imageCallback, this);
		gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
		
		//Service caller for returning a grid map
		ros::AdvertiseServiceOptions advertiseServiceOptionsForGetSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_submap", boost::bind(&ImageToGridmapDemo::getSubmap, this, _1, _2), ros::VoidConstPtr(),
      &serviceQueue_);
		submapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetSubmap);
	}

	ImageToGridmapDemo::~ImageToGridmapDemo()
	{
	}

	bool ImageToGridmapDemo::readParameters()
	{
		nodeHandle_.param("image_topic", imageTopic_, std::string("/image"));
		nodeHandle_.param("resolution", resolution_, 0.025);
		nodeHandle_.param("min_height", minHeight_, 0.0);
		nodeHandle_.param("max_height", maxHeight_, 0.2);
		return true;
	}

	void ImageToGridmapDemo::imageCallback(const sensor_msgs::Image& msg)
	{
		if (!mapInitialized_) {
			grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
			ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).",
				map_.getLength().x(),
				map_.getLength().y(),
				map_.getSize()(0),
				map_.getSize()(1));
			mapInitialized_ = true;
		}
		grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
		grid_map::GridMapRosConverter::addLayerFromImage(msg, "variance", map_, 0.0, 0.0);

		  // Publish as grid map.
		grid_map_msgs::GridMap mapMessage;
		grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
		gridMapPublisher_.publish(mapMessage);
	}
	
	bool ImageToGridmapDemo::getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
	{
		//atempt to simply pack and send existing map
		grid_map::GridMapRosConverter::toMessage(map_, response.map);
		
//		grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
//		Length requestedSubmapLength(request.length_x, request.length_y);
//		ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
//
//		bool computeSurfaceNormals = false;
//		if (request.layers.empty()) {
//			computeSurfaceNormals = true;
//		}
//		else {
//			for (const auto& type : request.layers) {
//				if (type.find("surface_normal") != std::string::npos) computeSurfaceNormals = true;
//			}
//		}
//
//		boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
//		map_.fuseArea(requestedSubmapPosition, requestedSubmapLength, computeSurfaceNormals);
//
//		bool isSuccess;
//		Index index;
//		GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
//		scopedLock.unlock();
//
//		if (request.layers.empty()) {
//			GridMapRosConverter::toMessage(subMap, response.map);
//		}
//		else {
//			vector<string> layers;
//			for (const auto& layer : request.layers) {
//				layers.push_back(layer);
//			}
//			GridMapRosConverter::toMessage(subMap, layers, response.map);
//		}
//
//		ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
//		return isSuccess;
		return true;
	}

} /* namespace */