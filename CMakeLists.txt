cmake_minimum_required(VERSION 2.8.3)
project(ardent)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  sensor_msgs
  std_msgs
)


find_package(Boost REQUIRED COMPONENTS system thread)
find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gazebo_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${gazebo_LIBRARY})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros
  DEPENDS 
    Boost
)

add_executable(walking_test test/walking_test.cpp)
target_link_libraries(walking_test ${catkin_LIBRARIES})
