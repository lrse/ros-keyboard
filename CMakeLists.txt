cmake_minimum_required(VERSION 2.8.3)
project(keyboard)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

set(LIBS ncurses)

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
  #INCLUDE_DIRS include
#  LIBRARIES keyboard
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib ${LIBS}
)

###########
## Build ##
###########

add_executable(keyboard src/main.cpp src/keyboard.cpp)

target_link_libraries(keyboard
  ${LIBS}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(TARGETS keyboard
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
