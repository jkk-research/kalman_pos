cmake_minimum_required(VERSION 3.8)
project(kalman_pos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(novatel_gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(INCLUDE_DIRS 
  include 
  ${ament_cmake_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  # ${novatel_gps_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

set(ament_dependencies
  # novatel_gps_msgs
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs  
)

add_library(PositionEstimation src/PositionEstimation.cpp ./include/PositionEstimation.h)
add_library(OrientationEstimation src/OrientationEstimation.cpp ./include/VehicleModelTypeDef.h)
add_library(CombinedVehicleModel src/CombinedVehicleModel.cpp ./include/VehicleModelTypeDef.h)
add_library(DynamicVehicleModel src/DynamicVehicleModel.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModel src/KinematicVehicleModel.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModel2 src/KinematicVehicleModel2.cpp ./include/VehicleModelTypeDef.h)
add_library(DynamicVehicleModelEKF src/DynamicVehicleModelEKF.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModelEKF src/KinematicVehicleModelEKF.cpp ./include/VehicleModelTypeDef.h)
add_library(DynamicVehicleModelEKFwoGNSS src/DynamicVehicleModelEKFwoGNSS.cpp ./include/VehicleModelTypeDef.h)
add_library(KinematicVehicleModelEKFwoGNSS src/KinematicVehicleModelEKFwoGNSS.cpp ./include/VehicleModelTypeDef.h)

target_link_libraries(CombinedVehicleModel PUBLIC DynamicVehicleModel KinematicVehicleModel KinematicVehicleModel2 DynamicVehicleModelEKF KinematicVehicleModelEKF DynamicVehicleModelEKFwoGNSS KinematicVehicleModelEKFwoGNSS)
target_link_libraries(PositionEstimation PUBLIC CombinedVehicleModel OrientationEstimation)

add_executable(kalman_pos_node src/kalman_pos_node.cpp)
ament_target_dependencies(kalman_pos_node  ${ament_dependencies} )
target_include_directories(kalman_pos_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kalman_pos_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(kalman_pos_node PositionEstimation)

add_executable(vehicle_status_convert src/VehicleStatusFromSteeringAndSpeed.cpp)
ament_target_dependencies(vehicle_status_convert  ${ament_dependencies} )
target_include_directories(vehicle_status_convert PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(vehicle_status_convert PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(vehicle_status_convert PositionEstimation)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  kalman_pos_node
  vehicle_status_convert
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
