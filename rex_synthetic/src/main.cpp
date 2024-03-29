/*
 * image_to_gridmap_demo_node.cpp
 *
 *  Created on: May 04, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "../include/rex_synthetic/rex_synthetic.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
	ros::init(argc, argv, "rex_synthetic");
	ros::NodeHandle nh("~");
	grid_map_demos::ImageToGridmapDemo imageToGridmapDemo(nh);

	ros::spin();
	return 0;
}