# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_msgs)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation geometry_msgs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  DIRECTORY msg
  FILES 
	Accelerometer.msg
	Color.msg
	Contact.msg
	Gyro.msg
	JointCommand.msg
	Range.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS std_msgs geometry_msgs
    INCLUDE_DIRS
    LIBRARIES
)