cmake_minimum_required(VERSION 2.8.3)
project(examples)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(OpenCV)
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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  message_generation
  roscpp
  random_numbers
  cv_bridge
  image_transport
  pcl_ros
  control_toolbox
  geometry_msgs
  nav_msgs
  sensor_msgs
)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
  mymsg.msg
  cmdmsg.msg
  sensmsg.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  mysrv.srv
  addsrv.srv
)

## Generate actions in the 'action' folder
add_action_files(FILES
  myact.action
  delayaction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES examples
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/examples.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/examples_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_examples.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(example1
  src/example1.cpp
)
add_dependencies(example1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example1
  ${catkin_LIBRARIES}
)

add_executable(example2
  src/example2.cpp
)
add_dependencies(example2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example2
  ${catkin_LIBRARIES}
)

add_executable(example3
  src/example3.cpp
)
add_dependencies(example3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example3
  ${catkin_LIBRARIES}
)

add_executable(example4
  src/example4.cpp
)
add_dependencies(example4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example4
  ${catkin_LIBRARIES}
)

add_executable(example5
  src/example5.cpp
)
add_dependencies(example5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example5
  ${catkin_LIBRARIES}
)

add_executable(example6
  src/example6.cpp
)
add_dependencies(example6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example6
  ${catkin_LIBRARIES}
)

add_executable(example7
  src/example7.cpp
)
add_dependencies(example7 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example7
  ${catkin_LIBRARIES}
)

add_executable(example8
  src/example8.cpp
)
add_dependencies(example8 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example8
  ${catkin_LIBRARIES}
)

add_executable(example9
  src/example9.cpp
)
add_dependencies(example9 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example9
  ${catkin_LIBRARIES}
)

add_executable(example10
  src/example10.cpp
)
add_dependencies(example10 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example10
  ${catkin_LIBRARIES}
)

add_executable(example11
  src/example11.cpp
)
add_dependencies(example11 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example11
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(example_eigen_basics
  src/example_eigen_basics.cpp
)
add_dependencies(example_eigen_basics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_basics
  ${catkin_LIBRARIES}
)

add_executable(example13
  src/example13.cpp
)
add_dependencies(example13 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example13
  ${catkin_LIBRARIES}
)

add_executable(example_cmd_line_params
  src/example_cmd_line_params.cpp
)
add_dependencies(example_cmd_line_params ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_cmd_line_params
  ${catkin_LIBRARIES}
)

add_executable(example_keyboard_input
  src/example_keyboard_input.cpp
)
add_dependencies(example_keyboard_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_keyboard_input
  ${catkin_LIBRARIES}
)

add_executable(example_pid_control
  src/example_pid_control.cpp
)
add_dependencies(example_pid_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pid_control
  ${catkin_LIBRARIES}
)

add_executable(example_random_number_generator
  src/example_random_number_generator.cpp
)
add_dependencies(example_random_number_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_random_number_generator
  ${catkin_LIBRARIES}
)

add_executable(example_resampling_algorithm
  src/example_resampling_algorithm.cpp
)
add_dependencies(example_resampling_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_resampling_algorithm
  ${catkin_LIBRARIES}
)

add_executable(example_robot_navigation_command
  src/example_robot_navigation_command.cpp
)
add_dependencies(example_robot_navigation_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_robot_navigation_command
  ${catkin_LIBRARIES}
)

add_executable(example_opencv_feature_detection
  src/example_opencv_feature_detection.cpp
)
add_dependencies(example_opencv_feature_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_opencv_feature_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(example_pcl2_point_cloud
  src/example_pcl2_point_cloud.cpp
)
add_dependencies(example_pcl2_point_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl2_point_cloud
  ${catkin_LIBRARIES}
)

add_executable(example_timer_callback
  src/example_timer_callback.cpp
)
add_dependencies(example_timer_callback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_timer_callback
  ${catkin_LIBRARIES}
)

add_executable(example_rviz_visulization_marker
  src/example_rviz_visulization_marker.cpp
)
add_dependencies(example_rviz_visulization_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_rviz_visulization_marker
  ${catkin_LIBRARIES}
)

add_executable(example_sinewave_generator
  src/example_sinewave_generator.cpp
)
add_dependencies(example_sinewave_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_sinewave_generator
  ${catkin_LIBRARIES}
)

add_executable(example_tf
  src/example_tf.cpp
)
add_dependencies(example_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_tf
  ${catkin_LIBRARIES}
)

add_executable(example_vrep_two_wheels_balancing_robot
  src/example_vrep_two_wheels_balancing_robot.cpp
)
add_dependencies(example_vrep_two_wheels_balancing_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_vrep_two_wheels_balancing_robot
  ${catkin_LIBRARIES}
)

add_executable(example_turtlesim_messages_services
  src/example_turtlesim_messages_services.cpp
)
add_dependencies(example_turtlesim_messages_services ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_turtlesim_messages_services
  ${catkin_LIBRARIES}
)

add_executable(example_action_client
  src/example_action_client.cpp
)
add_dependencies(example_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_client
  ${catkin_LIBRARIES}
)

add_executable(example_action_server
  src/example_action_server.cpp
)
add_dependencies(example_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_server
  ${catkin_LIBRARIES}
)

add_executable(example_service_client
  src/example_service_client.cpp
)
add_dependencies(example_service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_service_client
  ${catkin_LIBRARIES}
)

add_executable(example_service_server
  src/example_service_server.cpp
)
add_dependencies(example_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_service_server
  ${catkin_LIBRARIES}
)

add_executable(example_custom_message_publisher
  src/example_custom_message_publisher.cpp
)
add_dependencies(example_custom_message_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_custom_message_publisher
  ${catkin_LIBRARIES}
)

add_executable(example_custom_message_subscriber
  src/example_custom_message_subscriber.cpp
)
add_dependencies(example_custom_message_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_custom_message_subscriber
  ${catkin_LIBRARIES}
)

add_executable(example_vrep_demo_robot_controller
  src/example_vrep_demo_robot_controller.cpp
)
add_dependencies(example_vrep_demo_robot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_vrep_demo_robot_controller
  ${catkin_LIBRARIES}
)

add_executable(example_kalman_filter_state_estimation
  src/example_kalman_filter_state_estimation.cpp
)
add_dependencies(example_kalman_filter_state_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_kalman_filter_state_estimation
  ${catkin_LIBRARIES}
)

add_executable(example_particle_filter
  src/example_particle_filter.cpp
)
add_dependencies(example_particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_particle_filter
  ${catkin_LIBRARIES}
)

add_executable(example_kalman_filter_sensor_fusion
  src/example_kalman_filter_sensor_fusion.cpp
)
add_dependencies(example_kalman_filter_sensor_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_kalman_filter_sensor_fusion
  ${catkin_LIBRARIES}
)

add_executable(example_opencv_optical_flow
  src/example_opencv_optical_flow.cpp
)
add_dependencies(example_opencv_optical_flow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_opencv_optical_flow
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(example_structure_from_motion
  src/example_structure_from_motion.cpp
)
add_dependencies(example_structure_from_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_structure_from_motion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(example_occupancy_grid_map
  src/example_occupancy_grid_map.cpp
)
add_dependencies(example_occupancy_grid_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map
  ${catkin_LIBRARIES}
)

add_executable(example_laser_scan_fake
  src/example_laser_scan_fake.cpp
)
add_dependencies(example_laser_scan_fake ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_laser_scan_fake
  ${catkin_LIBRARIES}
)

add_executable(example_occupancy_grid_map_service_request
  src/example_occupancy_grid_map_service_request.cpp
)
add_dependencies(example_occupancy_grid_map_service_request ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map_service_request
  ${catkin_LIBRARIES}
)

add_executable(example_occupancy_grid_map_sample
  src/example_occupancy_grid_map_sample.cpp
)
add_dependencies(example_occupancy_grid_map_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map_sample
  ${catkin_LIBRARIES}
)

add_executable(example_tf_map_world
  src/example_tf_map_world.cpp
)
add_dependencies(example_tf_map_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_tf_map_world
  ${catkin_LIBRARIES}
)

add_executable(example_path_planning_dijkstra
  include/examples/Dijkstra.h
  src/Dijkstra.cpp
  src/example_path_planning_dijkstra.cpp
)
add_dependencies(example_path_planning_dijkstra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_path_planning_dijkstra
  ${catkin_LIBRARIES}
)

add_executable(example_path_planning_astar
  include/examples/AStar.h
  src/AStar.cpp
  src/example_path_planning_astar.cpp
)
add_dependencies(example_path_planning_astar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_path_planning_astar
  ${catkin_LIBRARIES}
)

add_executable(example_linear_least_square
  src/example_linear_least_square.cpp
)
add_dependencies(example_linear_least_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_linear_least_square
  ${catkin_LIBRARIES}
)

add_executable(example_least_square_accelerometer_error_parameters_estimation
  src/example_least_square_accelerometer_error_parameters_estimation.cpp
)
add_dependencies(example_least_square_accelerometer_error_parameters_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_least_square_accelerometer_error_parameters_estimation
  ${catkin_LIBRARIES}
)

add_executable(example_ransac_line_feature_estimate
  src/example_ransac_line_feature_estimate.cpp
)
add_dependencies(example_ransac_line_feature_estimate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_ransac_line_feature_estimate
  ${catkin_LIBRARIES}
)

add_executable(example_eigen_transformations
  src/example_eigen_transformations.cpp
)
add_dependencies(example_eigen_transformations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_transformations
  ${catkin_LIBRARIES}
)

add_executable(example_iterative_closest_points_algorithm
  src/example_iterative_closest_points_algorithm.cpp
)
add_dependencies(example_iterative_closest_points_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_iterative_closest_points_algorithm
  ${catkin_LIBRARIES}
)

add_executable(example_eigen_basics2
  src/example_eigen_basics2.cpp
)
add_dependencies(example_eigen_basics2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_basics2
  ${catkin_LIBRARIES}
)

add_executable(example_numerical_integration
  src/example_numerical_integration.cpp
)
add_dependencies(example_numerical_integration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_numerical_integration
  ${catkin_LIBRARIES}
)

add_executable(example_ray_tracing_3d
  src/example_ray_tracing_3d.cpp
)
add_dependencies(example_ray_tracing_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_ray_tracing_3d
  ${catkin_LIBRARIES}
)

add_executable(example_state_estimation_recursive_mean_covariance
  src/example_state_estimation_recursive_mean_covariance.cpp
)
add_dependencies(example_state_estimation_recursive_mean_covariance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_state_estimation_recursive_mean_covariance
  ${catkin_LIBRARIES}
)