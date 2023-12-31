cmake_minimum_required(VERSION 2.8.3)
project(cvssp_tools)

add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  image_geometry
  camera_calibration_parsers
  roscpp
  cv_bridge
  eigen_conversions
  geometry_msgs
  image_transport
  image_geometry
  laser_geometry
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf_conversions
)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

#Using FIND
#set(OpenCV_DIR $ENV{OpenCV_DIR})
#include(cmake/FindOpenCV.cmake)
### FIND THE UBUNTU VERSION
set(EXECUTE_COMMAND lsb_release -rs) 
execute_process(COMMAND ${EXECUTE_COMMAND} OUTPUT_VARIABLE UBUNTU_VERSION)
string(SUBSTRING ${UBUNTU_VERSION} 0 2 UBUNTU_MAJOR_VERSION)
if (${UBUNTU_MAJOR_VERSION} MATCHES "16") # if 16
  find_package(OpenCV REQUIRED)
else (${UBUNTU_MAJOR_VERSION MATCHES "14") # if 14 or less
  find_package(OpenCV REQUIRED)
endif (${UBUNTU_MAJOR_VERSION} MATCHES "16")
message( STATUS "OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS}) 

##Add OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -DHAVE_OPENMP")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OPENMP")
endif()

#Map Server Stuff
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    SemanticOccupancyGrid.msg
    SemanticScan.msg
)


# Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetMap.srv
   SetMap.srv
)

# Generate actions in the 'action' folder
add_action_files(
   FILES
   GetMap.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES cvssp_tools
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
    ${OpenCV_INCLUDE_DIRS} 
    ${BULLET_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(cvssp_tools
     src/Camera.cpp
     src/OfflineCamera.cpp
     src/Kinect/KinectSubscriber.cpp
     src/Kinect/KinectDepthLookUp.cpp
     src/RTabMapData.cpp #I dont think this is being used.
     src/depthToLaser/DepthImageToLaserScan.cpp
)
add_dependencies(cvssp_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cvssp_tools ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(cvssp_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(cvssp_tools_node src/cvssp_tools_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(cvssp_tools_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(cvssp_tools_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS cvssp_tools cvssp_tools_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cvssp_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(map_server
  src/map_server/map_server.cpp
  src/map_server/image_loader.cpp
)
add_dependencies(map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
#I dont think this is being used.
add_executable(main
  src/main.cpp
  src/Camera.cpp
  src/RTabMapData.cpp 
)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_executable(KinectSaver 
     src/Kinect/KinectImageSaver.cpp
     src/Camera.cpp
     src/Kinect/KinectSubscriber.cpp
     src/Kinect/KinectDepthLookUp.cpp)
add_dependencies(KinectSaver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KinectSaver ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_executable(KinectReader
     src/Kinect/KinectReader.cpp
     src/Kinect/KinectDepthLookUp.cpp)
add_dependencies(KinectReader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KinectReader ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_executable(mclpathvis
  src/PublishMCLPaths.cpp
)
add_dependencies(mclpathvis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mclpathvis ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})
