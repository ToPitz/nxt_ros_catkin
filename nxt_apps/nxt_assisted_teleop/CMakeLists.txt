# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_assisted_teleop)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS nxt_ros nxt_robot_sensor_car nxt_robot_gyro_car costmap_2d base_local_planner)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add_executable(assisted_teleop src/assisted_teleop.cpp)
add_executable(${PROJECT_NAME} src/assisted_teleop.cpp)

#common commands for building c++ executables and libraries
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS nxt_ros nxt_robot_sensor_car nxt_robot_gyro_car costmap_2d base_local_planner
    INCLUDE_DIRS
    LIBRARIES
)

## Install Python scripts
install(PROGRAMS scripts/range_to_pointcloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})