cmake_minimum_required(VERSION 2.8.3)
project(cvssp_amcl)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED
        COMPONENTS
            rosbag
            roscpp
            tf
            dynamic_reconfigure
            nav_msgs
            std_srvs
            cvssp_tools
        )

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

#Add OpenMP for Parallel
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -DHAVE_OPENMP")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OPENMP")
endif()

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
    CATKIN_DEPENDS
        rosbag
        roscpp
        dynamic_reconfigure
        tf
        cvssp_tools
  INCLUDE_DIRS include
  LIBRARIES cvssp_amcl_sensors cvssp_amcl_map cvssp_amcl_pf utily_funcs
)

include_directories(include/cvssp_amcl include/cvssp_amcl/map include/cvssp_amcl/sensors include/cvssp_amcl/pf)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(cvssp_amcl_pf
                    src/cvssp_amcl/pf/pf.c
                    src/cvssp_amcl/pf/pf_kdtree.c
                    src/cvssp_amcl/pf/pf_pdf.c
                    src/cvssp_amcl/pf/pf_vector.c
                    src/cvssp_amcl/pf/eig3.c
                    src/cvssp_amcl/pf/pf_draw.c)

add_library(cvssp_amcl_map
                    src/cvssp_amcl/map/map.c
                    src/cvssp_amcl/map/map_cspace.cpp
                    src/cvssp_amcl/map/map_range.c
                    src/cvssp_amcl/map/map_store.c
                    src/cvssp_amcl/map/map_draw.c
                    src/cvssp_amcl/map/map_draw_opencv.cpp)
target_link_libraries(cvssp_amcl_map ${OpenCV_LIBRARIES})

add_library(cvssp_amcl_sensors
                    src/cvssp_amcl/sensors/amcl_sensor.cpp
                    src/cvssp_amcl/sensors/amcl_odom.cpp
                    src/cvssp_amcl/sensors/amcl_laser.cpp
                    src/cvssp_amcl/sensors/amcl_semantic.cpp
                    src/cvssp_amcl/sensors/raman_lut.cpp
                    src/cvssp_amcl/sensors/wasserstein.h)

add_library(utily_funcs
            src/cvssp_amcl/sklearn_cpp/lsr.h
            src/cvssp_amcl/sklearn_cpp/lsr.cpp
            src/cvssp_amcl/peak_finder/PeakFinder.h
            src/cvssp_amcl/peak_finder/PeakFinder.cpp)

target_link_libraries(cvssp_amcl_sensors cvssp_amcl_map cvssp_amcl_pf utily_funcs ${OpenCV_LIBRARIES})


add_executable(cvssp_amcl
                       src/amcl_node.cpp)

add_dependencies(cvssp_amcl amcl_gencfg)

target_link_libraries(cvssp_amcl
    cvssp_amcl_sensors cvssp_amcl_map cvssp_amcl_pf utily_funcs
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    cvssp_amcl cvssp_amcl_sensors cvssp_amcl_map cvssp_amcl_pf utily_funcs
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/cvssp_amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

## Configure Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Bags
  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 41fe43af189ec71e5e48eb9ed661a655)
  catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 752f711cf4f6e8d1d660675e2da096b0)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
    http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
    http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 4a58d1a7962914009d99000d06e5939c)
  catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 6e3432115cccdca1247f6c807038e13d)
  catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 27deb742fdcd3af44cf446f39f2688a8)
  catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
    http://download.ros.org/data/amcl/rosie_localization_stage.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 3347bf3835724cfa45e958c5c1846066)

  # Maps
  catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
    http://download.ros.org/data/amcl/willow-full.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
    http://download.ros.org/data/amcl/willow-full-0.05.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b61694296e08965096c5e78611fd9765)

  # Tests
  add_rostest(test/set_initial_pose.xml)
  add_rostest(test/set_initial_pose_delayed.xml)
  add_rostest(test/basic_localization_stage.xml)
  add_rostest(test/small_loop_prf.xml)
  add_rostest(test/small_loop_crazy_driving_prg.xml)
  add_rostest(test/texas_greenroom_loop.xml)
  add_rostest(test/rosie_multilaser.xml)
  add_rostest(test/texas_willow_hallway_loop.xml)

# Not sure when or if this actually passed.
#
# The point of this is that you start with an even probability
# distribution over the whole map and the robot localizes itself after
# some number of iterations of sensing and motion.
#
#  add_rostest(test/global_localization_stage.xml)
endif()
