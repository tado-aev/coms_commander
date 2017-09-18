cmake_minimum_required(VERSION 2.8.3)
project(coms_commander)

find_package(catkin REQUIRED COMPONENTS
  coms_msgs
  roscpp
  nav_msgs
  geometry_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)


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
  INCLUDE_DIRS include
  LIBRARIES coms_commander
  CATKIN_DEPENDS coms_msgs roscpp nav_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(coms_commander
  src/coms_commander.cpp
)

add_executable(coms_commander_node src/coms_commander_node.cpp)

target_link_libraries(coms_commander_node
  coms_commander
  ${catkin_LIBRARIES}
)

install(TARGETS coms_commander coms_commander_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  configs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
