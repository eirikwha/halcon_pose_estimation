cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

project(halcon_pose_estimation)

## Use C++11
add_definitions(--std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    asr_halcon_bridge
    geometry_msgs
    kuka_eki_msgs
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    # ...
endif()

LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake matching_config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
#  LIBRARIES
    CATKIN_DEPENDS
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    geometry_msgs
    asr_halcon_bridge
    kuka_eki_msgs

#  DEPENDS
    DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    $ENV{HALCONROOT}/include
    /opt/halcon/include
    /opt/halcon/include/halconcpp
    ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/halcon_pose_estimation/*.cpp" "src/matching_config/*.cpp")

## Declare a C++ library
add_library(${PROJECT_NAME}
            ${SOURCES}
        )

install()
## Declare cpp executables
# Testing the functionality

add_executable(
        test_regular_matching
        src/test_regular_matching.cpp
)

add_executable(
        test_edge_matching
        src/test_edge_matching.cpp
)

add_executable(
        test_surface_model_creator
        src/test_surface_model_creator.cpp
)

## Specify libraries to link executable targets against
add_dependencies(test_regular_matching ${catkin_EXPORTED_TARGETS})
add_dependencies(test_edge_matching ${catkin_EXPORTED_TARGETS})
add_dependencies(test_surface_model_creator ${catkin_EXPORTED_TARGETS})


target_link_libraries(
        test_regular_matching
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        yaml-cpp
        halcon
        halconcpp
)

target_link_libraries(
        test_edge_matching
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        yaml-cpp
        halcon
        halconcpp
)

target_link_libraries(
        test_surface_model_creator
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        yaml-cpp
        halcon
        halconcpp
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )


