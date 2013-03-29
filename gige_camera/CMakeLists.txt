cmake_minimum_required(VERSION 2.8.3)
project(gige_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS camera_info_manager cv_bridge driver_base image_transport std_msgs timestamp_tools dynamic_reconfigure)
find_package( OpenCV REQUIRED )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


INCLUDE_DIRECTORIES(/opt/pleora/ebus_sdk/include)
LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib)
LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib/genicam/bin/Linux32_i86)

set(EBUS_LIB
	PvBuffer
	PvDevice
	PvGenICam
	PvGUI
	PvGUIUtils
	PvStream
	PvPersistence
	PvSerial
	PvStreamRaw
	PvTransmitterRaw
	PvVirtualDevice	
	PvBase
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

#generate_dynamic_reconfigure_options(
#	cfg/Flir.cfg 			
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

generate_dynamic_reconfigure_options(
    cfg/Flir.cfg
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gige_camera
#  CATKIN_DEPENDS camera_info_manager cv_bridge driver_base image_transport std_msgs timestamp_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
add_library(gige
  	src/libgige/gige.cpp
)

target_link_libraries(gige
	${EBUS_LIB}
	${OpenCV_LIBS}
)

## Declare a cpp executable
add_executable(flir_camera_node 
	src/nodes/flir_camera_node.cpp
)

## Add dependencies to the executable
#add_dependencies(flir_camera_node 
#	${PROJECT_NAME}
#	gige_camera_gencfg
#)

add_dependencies(flir_camera_node gige_camera_gencfg) 

## Specify libraries to link a library or executable target against
target_link_libraries(flir_camera_node
  	${catkin_LIBRARIES}
	gige
	${OpenCV_LIBS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS gige_camera gige_camera_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gige_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
