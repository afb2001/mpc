cmake_minimum_required(VERSION 2.8.3)
project(mpc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -funsafe-math-optimizations")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    geographic_msgs
    marine_msgs
    rosbag
    project11
    genmsg
    path_planner_common
    actionlib_msgs
    dynamic_reconfigure
    dubins_curves
    project11_transformations
    geographic_visualization_msgs
)

generate_dynamic_reconfigure_options(
        cfg/mpc.cfg
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp project11 path_planner_common
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${path_planner_INCLUDE_DIRS}
)

add_library(controller
        src/controller.cpp
        src/CurrentEstimator.cpp
        )

add_dependencies(controller path_planner_common project11_transformations_gencpp)

#target_link_libraries(controller path_planner_state dubins_plan dubins ${catkin_LIBRARIES})
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
        src/mpc_node.cpp
        )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp path_planner_common)
#add_dependencies(controller project11_transformations_gencpp path_planner_gencpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
        controller
        #path_planner_utilities
 )

catkin_add_gtest(test_controller test/test_controller.cpp test/NodeStub.cpp)

target_link_libraries(test_controller controller)

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )
