#
# R-PAD Laboratory
# Edward Ahn
#
cmake_minimum_required(VERSION 2.8.3)
project(occlusion_detection)

find_package(catkin REQUIRED
    COMPONENTS
    offscreen_render
    sensor_msgs
    )

catkin_package(
    CATKIN_DEPENDS
    offscreen_render
    sensor_msgs
    )
