cmake_minimum_required(VERSION 2.8.3)
project(ardent)

add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  sensor_msgs
  std_msgs
  Eigen3
  genmsg
  actionlib_msgs
  actionlib
)


find_package(Boost REQUIRED COMPONENTS system thread)
find_package(gazebo REQUIRED)

generate_messages(DEPENDENCIES actionlib_msgs)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${gazebo_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_NAME}
)

link_directories(${gazebo_LIBRARY} ${EIGEN3_INCLUDE_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS
    include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros
  DEPENDS 
    Boost
)

add_library(${PROJECT_NAME} 
            src/ardent.cc
            src/leg_kinematics.cc
            src/body_kinematics.cc)

add_executable(walking_test test/walking_test.cc)
target_link_libraries(walking_test ${catkin_LIBRARIES} ${PROJECT_NAME})
