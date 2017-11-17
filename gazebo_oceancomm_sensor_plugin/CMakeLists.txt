cmake_minimum_required(VERSION 2.8.3)
project(gazebo_oceancomm_sensor_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_plugins
    gazebo_ros
    roscpp
    )

find_package(gazebo REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES gazebo_oceancomm_sensor_plugin
    CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp
    #  DEPENDS system_lib
    )


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}/Paging
    )

# add_library(${PROJECT_NAME} src/oceancomm_sensor_plugin.cpp)
add_library(ArduCopterPlugin src/ArduCopterPlugin.cc)

## Specify libraries to link a library or executable target against
target_link_libraries( ArduCopterPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}  )
