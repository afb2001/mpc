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
    actionlib_msgs
    dynamic_reconfigure
)

add_service_files(
        FILES
        EstimateState.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        path_planner
)

generate_dynamic_reconfigure_options(
        cfg/mpc.cfg
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp project11 path_planner
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${pathplanner_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/mpc_node.cpp
        src/controller.cpp src/OtherCurrentEstimator.cpp src/OtherCurrentEstimator.h)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

catkin_add_gtest(test_controller test/test_controller.cpp src/controller.cpp test/NodeStub.cpp)

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )