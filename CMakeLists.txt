cmake_minimum_required(VERSION 3.0.2)
project(apsrc_lead_vehicle_detection)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
    apsrc_msgs
    autoware_msgs
    geometry_msgs
    nodelet
    roscpp
    roslib
    roslint
    std_msgs
    radar_msgs
    derived_object_msgs
    tf2_ros
)

find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    LIBRARIES ${PROJECT_NAME}_nodelets
)

roslint_cpp()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# nodelets
add_library(${PROJECT_NAME}_nodelets
    src/apsrc_lead_vehicle_detection_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelets
    ${catkin_LIBRARIES}
)

# nodes
add_executable(${PROJECT_NAME}_node
    src/apsrc_lead_vehicle_detection_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

############
# Install ##
############

install(TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelets
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

