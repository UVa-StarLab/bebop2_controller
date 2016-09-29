cmake_minimum_required(VERSION 2.8.3)
project(bebop2_controller)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    nav_msgs
    std_msgs
    roscpp
    tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab01
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(lab01
#   src/${PROJECT_NAME}/lab01.cpp
# )

## Declare a cpp executable
#add_executable(PID_node src/PID_control.cpp)
add_executable(movement_limiter_node src/bebop_movement_limiter.cpp)

## as an example, message headers may need to be generated before nodes
##add_dependencies(movement_limiter_node vel_control_node_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(PID_node
#  ${catkin_LIBRARIES}
#)
target_link_libraries(movement_limiter_node
  ${catkin_LIBRARIES}
)
