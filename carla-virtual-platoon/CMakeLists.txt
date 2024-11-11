cmake_minimum_required(VERSION 3.8)
project(carla-virtual-platoon)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wl,-Bstatic -lrpc -lboost_filesystem -Wl,-Bdynamic -lpng -ltiff -ljpeg -lRecast -lDetour -lDetourCrowd)
endif()

if(NOT DEFINED ENV{CARLA_ROOT})
  message(FATAL_ERROR "Environment variable CARLA_ROOT is not set. Please set CARLA_ROOT to the root directory of your CARLA installation.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(TIFF REQUIRED)
find_package(ros2_msg REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(ament_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  Boost
  ros2_msg
)

##############################################################################################
include(dependency.cmake)

add_executable(main
    src/main.cpp
    nodes/Spectator.cpp
    nodes/Camera.cpp
    nodes/Radar.cpp
    nodes/Lidar.cpp
    nodes/TruckControl.cpp
    nodes/TruckStatus.cpp
    nodes/TruckOBU.cpp
)

##############################################################################################
ament_target_dependencies(main ${ament_dependencies})

target_include_directories(main PUBLIC
  ${libCarla_INCLUDE_DIR}
  ${DEP_INCLUDE_DIR}
)
target_include_directories(main PUBLIC 
  ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(main PUBLIC
  ${libCarla_LIB_DIR}
)

target_link_libraries(main 
  ${libCarla_LIBS} 
  ${Boost_LIBRARIES} 
  ${TIFF_LIBRARIES} 
  jpeg png rt
)
##############################################################################################

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ${libCarla_LIB_DIR}/ 
  DESTINATION lib
)

ament_package()
