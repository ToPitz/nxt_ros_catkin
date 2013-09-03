# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_robot_gyro_car)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS nxt_ros nxt_controllers robot_pose_ekf nxt_description robot_state_publisher)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS nxt_ros nxt_controllers robot_pose_ekf nxt_description robot_state_publisher
    INCLUDE_DIRS
    LIBRARIES
)