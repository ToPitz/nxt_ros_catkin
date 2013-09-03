# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nxt_description)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Download test data - TODO: How to untar?
#catkin_download_test_data(
#  ${PROJECT_NAME}/meshes/nxt_description.tar.gz
#  http://foote-ros-pkg.googlecode.com/files/nxt_description.parts.tar.gz 
#  MD5 5b1eb23778428d33f8f6462e41d5c5e8)
#rosbuild_untar_file(meshes/nxt_description.tar.gz meshes/parts ALL
  
#rosbuild_download_data(http://foote-ros-pkg.googlecode.com/files/nxt_description.parts.tar.gz meshes/nxt_description.tar.gz 5b1eb23778428d33f8f6462e41d5c5e8)
#rosbuild_untar_file(meshes/nxt_description.tar.gz meshes/parts ALL)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)