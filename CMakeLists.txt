cmake_minimum_required(VERSION 3.5)
project(octomap_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  message_filters
  octomap_ros
  octomap_msgs
  tf2
  tf2_ros
  tf2_msgs
  mrs_lib
  laser_geometry
  nodelet
  )

add_definitions("-fdiagnostics-color=always -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_compile_options("-Wall")
add_compile_options("-Wextra")
add_compile_options(${PCL_COMPILE_OPTIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenMP REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS mrs_msgs
)

# include include
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

add_library(OctomapConversions SHARED
  src/conversions.cpp
  )
add_dependencies(OctomapConversions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(OctomapConversions
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OCTOMAP_LIBRARIES}
  )

add_library(octomap_server SHARED
  src/octomap_server.cpp
  )
add_dependencies(octomap_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(octomap_server
  OctomapConversions
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${OCTOMAP_LIBRARIES}
  )
