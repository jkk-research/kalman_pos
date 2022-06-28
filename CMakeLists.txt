cmake_minimum_required(VERSION 3.0.2)
project(kalman_pos)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -o2)

# set(CMAKE_PREFIX_PATH _PATH "/opt/ros/melodic;${CMAKE_MODULE_PATH}")
 
# find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})
 
# find_package(Boost REQUIRED COMPONENTS system)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  autoware_msgs
  novatel_gps_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marathon_waypoint
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#set(HEADERS
#    ${HEADERS}
#    include/VehicleModelTypeDef.h
#    include/CombinedVehicleModel.h
#    include/DynamicVehicleModel.h
#    include/KinematicVehicleModel.h
#    include/DynamicVehicleModelEKF.h
#    include/KinematicVehicleModelEKF.h
#    include/DynamicVehicleModelEKFwoGNSS.h
#    include/KinematicVehicleModelEKFwoGNSS.h
#)
# 
# # Local source files here
# SET(SOURCES
#     ${SOURCES}
#     src/CombinedVehicleModel.cpp
#     src/DynamicVehicleModel.cpp
#     src/KinematicVehicleModel.cpp
#     )

add_library(CombinedVehicleModel src/CombinedVehicleModel.cpp ./include/VehicleModelTypeDef.h)
add_library(DynamicVehicleModel src/DynamicVehicleModel.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModel src/KinematicVehicleModel.cpp ./include/VehicleModelTypeDef.h)
add_library(DynamicVehicleModelEKF src/DynamicVehicleModelEKF.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModelEKF src/KinematicVehicleModelEKF.cpp ./include/VehicleModelTypeDef.h)
add_library(DynamicVehicleModelEKFwoGNSS src/DynamicVehicleModelEKFwoGNSS.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModelEKFwoGNSS src/KinematicVehicleModelEKFwoGNSS.cpp ./include/VehicleModelTypeDef.h)

target_link_libraries(CombinedVehicleModel PUBLIC DynamicVehicleModel KinematicVehicleModel DynamicVehicleModelEKF KinematicVehicleModelEKF DynamicVehicleModelEKFwoGNSS KinematicVehicleModelEKFwoGNSS)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/marathon_waypoint.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(kalman_pos_node src/kalman_pos_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kalman_pos_node CombinedVehicleModel ${catkin_LIBRARIES})

## The recommended prefix ensures that target names across packages don't collide
add_executable(kalman_pos_no_aw src/kalman_pos_no_aw.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(kalman_pos_no_aw ${catkin_LIBRARIES})
