cmake_minimum_required(VERSION 2.8.3)
project(ardent)

add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  sensor_msgs
  std_msgs
  genmsg
  actionlib_msgs
  actionlib
  pluginlib
  angles
  hardware_interface
  urdf
  cmake_modules
)
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(gazebo REQUIRED)
find_package(PkgConfig)

pkg_check_modules(URDFDOM REQUIRED urdfdom)
if ("${URDFDOM_VERSION}" VERSION_EQUAL "1.0.0" OR "${URDFDOM_VERSION}" VERSION_GREATER "1.0.0")
  add_definitions(-DURDFDOM_1_0_0_API)
endif()

# generate_messages(DEPENDENCIES actionlib_msgs)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${gazebo_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_NAME}
  ${urdfdom_INCLUDE_DIRS}
)

link_directories(${gazebo_LIBRARY} ${EIGEN3_INCLUDE_DIR})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


set(LIB_SOURCES
    src/robot.cc
    src/leg_kinematics.cc
    src/body_kinematics.cc
    src/hardware_interface.cc
    src/simple_transmission.cc
    src/joint.cpp)

add_library(${PROJECT_NAME} ${LIB_SOURCES})

catkin_package(
  INCLUDE_DIRS
    include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros
    hardware_interface
    pluginlib
    urdf
    cmake_modules
  DEPENDS 
    Boost
    TinyXML
    urdfdom
)

add_executable(walking_test test/walking_test.cc)
target_link_libraries(walking_test ${catkin_LIBRARIES} ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}  ltdl ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE)

install(TARGETS ardent
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    robot/params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)