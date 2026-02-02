cmake_minimum_required(VERSION 3.0.2)
project(makinami)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  std_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES makinami
 CATKIN_DEPENDS roscpp pcl_ros std_msgs
#  DEPENDS system_lib
)

include_directories(
include
${Boost_INCLUDE_DIRS}
${Eigen3_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(makinami_nodelet apps/makinami_nodelet.cpp)
target_include_directories(makinami_nodelet PRIVATE
  include
  ${CATKIN_DEVEL_PREFIX}/include
)
target_link_libraries(makinami_nodelet
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
