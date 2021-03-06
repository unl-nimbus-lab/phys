cmake_minimum_required(VERSION 2.8.3)
project(lama_jockeys)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(ROS_DEPENDENCIES
  actionlib
  actionlib_msgs
  geometry_msgs
  lama_interfaces
  lama_msgs
  message_generation
  roscpp
  )
find_package(catkin REQUIRED COMPONENTS
  ${ROS_DEPENDENCIES}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Learn.action
  Localize.action
  Navigate.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  lama_msgs
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
  INCLUDE_DIRS
  include

  LIBRARIES
  jockey
  learning_jockey
  localizing_jockey
  navigating_jockey

  CATKIN_DEPENDS
  ${ROS_DEPENDENCIES}
  
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(jockey src/jockey.cpp)
add_library(learning_jockey src/learning_jockey.cpp)
add_library(localizing_jockey src/localizing_jockey.cpp)
add_library(navigating_jockey src/navigating_jockey.cpp)

## Declare a cpp executable
# add_executable(lama_jockeys_node src/lama_jockeys_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(jockey lama_jockeys_generate_messages_cpp)
add_dependencies(learning_jockey lama_jockeys_generate_messages_cpp)
add_dependencies(learning_jockey jockey)
add_dependencies(localizing_jockey lama_jockeys_generate_messages_cpp)
add_dependencies(localizing_jockey jockey)
add_dependencies(navigating_jockey lama_jockeys_generate_messages_cpp)
add_dependencies(navigating_jockey jockey)

## Specify libraries to link a library or executable target against
target_link_libraries(jockey ${catkin_LIBRARIES})
target_link_libraries(learning_jockey jockey ${catkin_LIBRARIES})
target_link_libraries(localizing_jockey jockey ${catkin_LIBRARIES})
target_link_libraries(navigating_jockey jockey ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS jockey learning_jockey localizing_jockey navigating_jockey
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lama_jockeys.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
