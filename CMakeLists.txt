cmake_minimum_required(VERSION 2.8.3)
project(mpc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    geographic_msgs
    marine_msgs
    rosbag
    project11
    genmsg
    path_planner
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp project11 path_planner
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mpc_node.cpp src/controller.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )