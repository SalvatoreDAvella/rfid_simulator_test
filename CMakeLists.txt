cmake_minimum_required(VERSION 3.0.2)
project(rfid_simulator_test)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rfid_sensor
  roscpp
  geometry_msgs
  sensor_msgs
)

find_package(gazebo REQUIRED)
message(${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS gazebo_ros roscpp
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(collect_data_node src/collect_data.cpp)
target_link_libraries(collect_data_node
  ${catkin_LIBRARIES}
)

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(move_antenna SHARED src/moveAntenna.cpp)
target_include_directories(move_antenna SYSTEM PRIVATE include ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(move_antenna ${GAZEBO_LIBRARIES})
