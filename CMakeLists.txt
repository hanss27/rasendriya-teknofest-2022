cmake_minimum_required(VERSION 3.0.2)
project(rasendriya)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  mavros
  message_generation
  roslaunch
  sensor_msgs
)

## Check all *.launch files in launch directory
roslaunch_add_file_check(launch)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(FILES
    Dropzone.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
    std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rasendriya.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(mission_control
    src/mission_control.cpp)
target_link_libraries(mission_control
    ${catkin_LIBRARIES})

add_executable(test_servo
    src/testing/test_servo.cpp)
target_link_libraries(test_servo
    ${catkin_LIBRARIES})

add_executable(test_waypoint
    src/testing/test_waypoint.cpp)
target_link_libraries(test_waypoint
    ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
    src/vision_programs/vision_dropzone.py   
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS mission_control test_servo test_waypoint
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE
)
