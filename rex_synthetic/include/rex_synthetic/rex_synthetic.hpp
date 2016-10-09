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

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>

#include <string>

namespace grid_map_demos {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
	class ImageToGridmapDemo
	{
	public:

	  /*!
	   * Constructor.
	   * @param nodeHandle the ROS node handle.
	   */
		ImageToGridmapDemo(ros::NodeHandle& nodeHandle);

		  /*!
		   * Destructor.
		   */
		virtual ~ImageToGridmapDemo();

		  /*!
		  * Reads and verifies the ROS parameters.
		  * @return true if successful.
		  */
		bool readParameters();

		void imageCallback(const sensor_msgs::Image& msg);

	private:

	  //! ROS nodehandle.
		ros::NodeHandle& nodeHandle_;

		  //! Grid map publisher.
		ros::Publisher gridMapPublisher_;

		  //! Grid map data.
		grid_map::GridMap map_;

		  //! Image subscriber
		ros::Subscriber imageSubscriber_;

		  //! Name of the grid map topic.
		std::string imageTopic_;

		  //! Length of the grid map in x direction.
		double mapLengthX_;

		  //! Resolution of the grid map.
		double resolution_;

		  //! Range of the height values.
		double minHeight_;
		double maxHeight_;

		  //! Frame id of the grid map.
		std::string mapFrameId_;
		
		//added for service calls
		ros::ServiceServer	submapService_;
		ros::CallbackQueue  serviceQueue_;

		bool mapInitialized_;
		
	public:
		bool getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
	};

} /* namespace */