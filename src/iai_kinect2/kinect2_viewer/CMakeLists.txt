cmake_minimum_required(VERSION 2.8.3)
project(kinect2_viewer CXX)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for c++11 support检查是否支持C++11
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs
    message_filters
    cv_bridge
    image_transport
    compressed_image_transport
    compressed_depth_image_transport
    kinect2_bridge
    )

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

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
#  LIBRARIES kinect2_viewer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
)

# -----------------SRC_VIEWER-----------------
add_executable(kinect2_viewer src/viewer.cpp)
target_link_libraries(kinect2_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
)
# -----------------SRC-----------------
#    add_executable(pcl_cloud_viewer src/pcl_cloud_viewer.cpp)
#    target_link_libraries(pcl_cloud_viewer ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

#    add_executable(image_viewer src/image_viewer.cpp)
#    target_link_libraries(image_viewer ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

#    add_executable(pcl_read src/pcl_read.cpp)
#    target_link_libraries(pcl_read ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

#    add_executable(image_converter src/image_converter.cpp)
#    target_link_libraries(image_converter ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

# -----------------SOURCE-----------------
## --read
    add_executable(cloud_read source/cloud_read.cpp)
    target_link_libraries(cloud_read ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

## --viewer
#    add_executable(view_Normal source/view_Normal.cpp)
#    target_link_libraries(view_Normal ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

    add_executable(viewer_process source/viewer_process.cpp)
    target_link_libraries(viewer_process ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

## --segment
#    add_executable(seg_euclidean_cluster source/seg_euclidean_cluster.cpp)
#    target_link_libraries(seg_euclidean_cluster ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

#    add_executable(seg_cylinder source/seg_cylinder.cpp)
#    target_link_libraries(seg_cylinder ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

#    add_executable(seg_cylinder_no_class source/seg_cylinder_no_class.cpp)
#    target_link_libraries(seg_cylinder_no_class ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

## --feature
#    add_executable(fealture_normalEstimation source/fealture_normalEstimation.cpp)
#    target_link_libraries(fealture_normalEstimation ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

# -----------------6DOF-----------------
# --filter
    add_executable(filter_cloud_6dof 6DOF/filter_cloud_6dof.cpp)
    target_link_libraries(filter_cloud_6dof ${catkin_LIBRARIES}  ${PCL_LIBRARIES})


    add_executable(seg_cylinder_6dof 6DOF/seg_cylinder_6dof.cpp)
    target_link_libraries(seg_cylinder_6dof ${catkin_LIBRARIES}  ${PCL_LIBRARIES})


    add_executable(viewer_6dof 6DOF/viewer_6dof.cpp)
    target_link_libraries(viewer_6dof ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

    add_executable(viewer_6dof2 6DOF/viewer_6dof2.cpp)
    target_link_libraries(viewer_6dof2 ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

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
install(TARGETS kinect2_viewer
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinect2_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
