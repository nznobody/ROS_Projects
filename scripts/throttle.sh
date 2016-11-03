#!/bin/bash

cd ~/rex_ws
source devel/setup.bash

rosrun topic_tools throttle messages /camera/rgb/image_rect_color 1 &
rosrun topic_tools throttle messages /rex_gridmapping/elevation_map_raw 1 &
rosrun topic_tools throttle messages /rex_traversibility/traversability_map 1 &