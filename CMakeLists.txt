cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


add_definitions(-msse -msse2)
add_definitions(-DEIGEN_NO_DEBUG)
add_definitions(-DNDEBUG)

rosbuild_add_library(trajectory_optimizer
	src/trajectory_cost.cpp
	src/trajectory_optimizer.cpp
	src/trajectory_parameters.cpp
	src/trajectory_optimizer_node.cpp
	src/trajectory.cpp
)	

rosbuild_add_executable(trajectory_optimizer_node
	src/trajectory_optimizer_node.cpp
)
target_link_libraries(trajectory_optimizer_node trajectory_optimizer)


