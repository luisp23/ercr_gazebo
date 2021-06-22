cmake_minimum_required(VERSION 3.0.2)
project(ercr_gazebo)

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(roscpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS ercr_msgs sensor_msgs )
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(sensor_msgs REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ercr_msgs
  LIBRARIES 
  DEPENDS sensor_msgs
)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}

)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
)


add_subdirectory(msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

############ Thrust plugin ###################################
###############################################################
add_library(thrust_plugin SHARED src/thrust_plugin.cpp)

target_link_libraries(thrust_plugin 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ercr_msgs
)

add_dependencies(thrust_plugin
  ${catkin_EXPORTED_TARGETS}
  ercr_msgs
)

############ Sensors plugin ###################################
###############################################################
add_library(sensors_plugin SHARED src/sensors_plugin.cpp)

target_link_libraries(sensors_plugin 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(sensors_plugin
  ${catkin_EXPORTED_TARGETS}
)