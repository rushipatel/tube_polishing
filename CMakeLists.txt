cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#common commands for building c++ executables and libraries
rosbuild_add_library(tube_manipulation src/tube_manipulation.cpp src/robot_head.cpp)
#rosbuild_add_library(tube_manipulation_lib src/collision_check.cpp)
rosbuild_add_library(tube_perception src/tube_perception.cpp)
rosbuild_add_library(tube_grasp src/tube_grasp.cpp)
#rosbuild_add_library(control_seq src/control_sequence.cpp)
rosbuild_add_library(state_machine src/state_machine.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/main.cpp)
#rosbuild_add_executable(tube_tabletop_segmentation src/tabletop_segmentation.cpp)
#target_link_libraries(${PROJECT_NAME} control_seq)
target_link_libraries(${PROJECT_NAME} tube_manipulation)
target_link_libraries(${PROJECT_NAME} tube_perception)
target_link_libraries(${PROJECT_NAME} tube_grasp)
target_link_libraries(${PROJECT_NAME} state_machine)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
include_directories(/usr/include/vtk-5.8)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
