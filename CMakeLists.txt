cmake_minimum_required(VERSION 3.10)
project(PCL_Project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add executables
add_executable(object_detection_node src/object_detection.cpp)
target_link_libraries(object_detection_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pcl_preprocessing_node src/pcl_preprocessing.cpp)
target_link_libraries(pcl_preprocessing_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# Catkin package
catkin_package()