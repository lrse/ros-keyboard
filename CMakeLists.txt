cmake_minimum_required(VERSION 2.8.3)
project(keyboard)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED SDL2)
PKG_SEARCH_MODULE(SDL2TTF REQUIRED SDL2_ttf)
set(SDL2_LIBRARIES ${SDL2_LIBRARY} ${SDL2TTF_LIBRARIES})
set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS} ${SDL2TTF_INCLUDE_DIRS})

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Key.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS SDL2 SDL2TTF
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS})

add_executable(keyboard src/main.cpp src/keyboard.cpp)

target_link_libraries(keyboard
  ${SDL2_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(keyboard keyboard_gencpp)

#############
## Install ##
#############


install(TARGETS keyboard
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
