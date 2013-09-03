# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_controllers)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs tf tf_conversions nxt_msgs nav_msgs)
find_package(orocos_kdl REQUIRED)

#include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS kdl
    CATKIN_DEPENDS rospy sensor_msgs tf tf_conversions nxt_msgs nav_msgs
    INCLUDE_DIRS
    LIBRARIES
)

## Install Python scripts
install(PROGRAMS
  scripts/base_controller.py
  scripts/base_odometry.py
  scripts/joint_position_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)