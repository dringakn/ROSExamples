cmake_minimum_required(VERSION 3.0.2)

project(ros_examples) # Package name

add_compile_options(-std=c++17) # C++ Compiler Standards

find_package(catkin REQUIRED) # ROS build system
find_package(OpenCV REQUIRED) # Open CV
find_package(PCL REQUIRED) # PCL
find_package(octomap REQUIRED) # OctoMap
find_package(Boost REQUIRED) # Boost libraries

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  message_generation
  roscpp
  random_numbers
  cv_bridge
  image_transport
  pcl_conversions
  control_toolbox
  geometry_msgs
  nav_msgs
  sensor_msgs
  move_base_msgs
  octomap_ros
  octomap_msgs
  pcl_ros
  costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  # pcl_msgs
  rviz
  std_msgs
  roscpp
)

# RViz Display plugin related libraries
message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets) # Likner error
set(CMAKE_AUTOMOC ON) # Unknown symbol error

# catkin_package() #RViz plugin

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

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

catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES ros_examples
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
add_library(example_rviz_display src/example_rviz_display.cpp)
add_dependencies(example_rviz_display ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_rviz_display ${catkin_LIBRARIES} ${QT_LIBRARIES} )

add_executable(example_simple_ros_cpp_node  src/example_simple_ros_cpp_node.cpp)
add_dependencies(example_simple_ros_cpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_simple_ros_cpp_node  ${catkin_LIBRARIES})

add_executable(example_random_number_topic  src/example_random_number_topic.cpp)
add_dependencies(example_random_number_topic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_random_number_topic  ${catkin_LIBRARIES})

add_executable(example_custom_messages_publisher  src/example_custom_messages_publisher.cpp)
add_dependencies(example_custom_messages_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_custom_messages_publisher  ${catkin_LIBRARIES})

add_executable(example_custom_messages_subscriber  src/example_custom_messages_subscriber.cpp)
add_dependencies(example_custom_messages_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_custom_messages_subscriber  ${catkin_LIBRARIES})

add_executable(example_parameter_server  src/example_parameter_server.cpp)
add_dependencies(example_parameter_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_parameter_server  ${catkin_LIBRARIES})

add_executable(example_service_client_add  src/example_service_client_add.cpp)
add_dependencies(example_service_client_add ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_service_client_add  ${catkin_LIBRARIES})

add_executable(example_service_server_add  src/example_service_server_add.cpp)
add_dependencies(example_service_server_add ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_service_server_add  ${catkin_LIBRARIES})

add_executable(example_action_client_simple  src/example_action_client_simple.cpp)
add_dependencies(example_action_client_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_client_simple  ${catkin_LIBRARIES})

add_executable(example_timer_simple  src/example_timer_simple.cpp)
add_dependencies(example_timer_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_timer_simple  ${catkin_LIBRARIES})

add_executable(example_opencv_capture_image  src/example_opencv_capture_image.cpp)
add_dependencies(example_opencv_capture_image ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_opencv_capture_image  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(example_eigen_basics  src/example_eigen_basics.cpp)
add_dependencies(example_eigen_basics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_basics  ${catkin_LIBRARIES})

add_executable(example_pcl_load_ply_file  src/example_pcl_load_ply_file.cpp)
add_dependencies(example_pcl_load_ply_file ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_load_ply_file  ${catkin_LIBRARIES})

add_executable(example_cmd_line_params  src/example_cmd_line_params.cpp)
add_dependencies(example_cmd_line_params ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_cmd_line_params  ${catkin_LIBRARIES})

add_executable(example_keyboard_input  src/example_keyboard_input.cpp)
add_dependencies(example_keyboard_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_keyboard_input  ${catkin_LIBRARIES})

add_executable(example_pid_control  src/example_pid_control.cpp)
add_dependencies(example_pid_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pid_control  ${catkin_LIBRARIES})

add_executable(example_random_number_generator  src/example_random_number_generator.cpp)
add_dependencies(example_random_number_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_random_number_generator  ${catkin_LIBRARIES})

add_executable(example_resampling_algorithm  src/example_resampling_algorithm.cpp)
add_dependencies(example_resampling_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_resampling_algorithm  ${catkin_LIBRARIES})

add_executable(example_robot_navigation_command  src/example_robot_navigation_command.cpp)
add_dependencies(example_robot_navigation_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_robot_navigation_command  ${catkin_LIBRARIES})

add_executable(example_opencv_feature_detection  src/example_opencv_feature_detection.cpp)
add_dependencies(example_opencv_feature_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_opencv_feature_detection  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(example_pcl2_point_cloud  src/example_pcl2_point_cloud.cpp)
add_dependencies(example_pcl2_point_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl2_point_cloud  ${catkin_LIBRARIES})

add_executable(example_timer_callback  src/example_timer_callback.cpp)
add_dependencies(example_timer_callback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_timer_callback  ${catkin_LIBRARIES})

add_executable(example_rviz_visulization_marker  src/example_rviz_visulization_marker.cpp)
add_dependencies(example_rviz_visulization_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_rviz_visulization_marker  ${catkin_LIBRARIES})

add_executable(example_sinewave_generator  src/example_sinewave_generator.cpp)
add_dependencies(example_sinewave_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_sinewave_generator  ${catkin_LIBRARIES})

add_executable(example_tf  src/example_tf.cpp)
add_dependencies(example_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_tf  ${catkin_LIBRARIES})

add_executable(example_tf2_gps src/example_tf2_gps.cpp)
add_dependencies(example_tf2_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_tf2_gps ${catkin_LIBRARIES})

add_executable(example_vrep_two_wheels_balancing_robot  src/example_vrep_two_wheels_balancing_robot.cpp)
add_dependencies(example_vrep_two_wheels_balancing_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_vrep_two_wheels_balancing_robot  ${catkin_LIBRARIES})

add_executable(example_turtlesim_messages_services  src/example_turtlesim_messages_services.cpp)
add_dependencies(example_turtlesim_messages_services ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_turtlesim_messages_services  ${catkin_LIBRARIES})

add_executable(example_action_client  src/example_action_client.cpp)
add_dependencies(example_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_client  ${catkin_LIBRARIES})

add_executable(example_action_server  src/example_action_server.cpp)
add_dependencies(example_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_server  ${catkin_LIBRARIES})

add_executable(example_service_client  src/example_service_client.cpp)
add_dependencies(example_service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_service_client  ${catkin_LIBRARIES})

add_executable(example_service_server  src/example_service_server.cpp)
add_dependencies(example_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_service_server  ${catkin_LIBRARIES})

add_executable(example_custom_message_publisher  src/example_custom_message_publisher.cpp)
add_dependencies(example_custom_message_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_custom_message_publisher  ${catkin_LIBRARIES})

add_executable(example_custom_message_subscriber  src/example_custom_message_subscriber.cpp)
add_dependencies(example_custom_message_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_custom_message_subscriber  ${catkin_LIBRARIES})

add_executable(example_vrep_demo_robot_controller  src/example_vrep_demo_robot_controller.cpp)
add_dependencies(example_vrep_demo_robot_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_vrep_demo_robot_controller  ${catkin_LIBRARIES})

add_executable(example_kalman_filter_state_estimation  src/example_kalman_filter_state_estimation.cpp)
add_dependencies(example_kalman_filter_state_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_kalman_filter_state_estimation  ${catkin_LIBRARIES})

add_executable(example_particle_filter  src/example_particle_filter.cpp)
add_dependencies(example_particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_particle_filter  ${catkin_LIBRARIES})

add_executable(example_kalman_filter_sensor_fusion  src/example_kalman_filter_sensor_fusion.cpp)
add_dependencies(example_kalman_filter_sensor_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_kalman_filter_sensor_fusion  ${catkin_LIBRARIES})

add_executable(example_opencv_optical_flow  src/example_opencv_optical_flow.cpp)
add_dependencies(example_opencv_optical_flow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_opencv_optical_flow  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(example_structure_from_motion  src/example_structure_from_motion.cpp)
add_dependencies(example_structure_from_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_structure_from_motion  ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(example_costmap_2d_basic  src/example_costmap_2d_basic.cpp)
add_dependencies(example_costmap_2d_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_costmap_2d_basic  ${catkin_LIBRARIES})

add_executable(example_occupancy_grid_map  src/example_occupancy_grid_map.cpp)
add_dependencies(example_occupancy_grid_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map  ${catkin_LIBRARIES})

add_executable(example_occupancy_grid_map_server  src/example_occupancy_grid_map_server.cpp)
add_dependencies(example_occupancy_grid_map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map_server  ${catkin_LIBRARIES})

add_executable(example_laser_scan_fake  src/example_laser_scan_fake.cpp)
add_dependencies(example_laser_scan_fake ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_laser_scan_fake  ${catkin_LIBRARIES})

add_executable(example_occupancy_grid_map_service_request  src/example_occupancy_grid_map_service_request.cpp)
add_dependencies(example_occupancy_grid_map_service_request ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map_service_request  ${catkin_LIBRARIES})

add_executable(example_occupancy_grid_map_sample  src/example_occupancy_grid_map_sample.cpp)
add_dependencies(example_occupancy_grid_map_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_occupancy_grid_map_sample  ${catkin_LIBRARIES})

add_executable(example_tf_map_world  src/example_tf_map_world.cpp)
add_dependencies(example_tf_map_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_tf_map_world  ${catkin_LIBRARIES})

add_executable(example_path_planning_dijkstra  include/Dijkstra.h  src/Dijkstra.cpp  src/example_path_planning_dijkstra.cpp)
add_dependencies(example_path_planning_dijkstra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_path_planning_dijkstra  ${catkin_LIBRARIES})

add_executable(example_path_planning_astar  include/AStar.h  src/AStar.cpp  src/example_path_planning_astar.cpp)
add_dependencies(example_path_planning_astar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_path_planning_astar  ${catkin_LIBRARIES})

add_executable(example_linear_least_square  src/example_linear_least_square.cpp)
add_dependencies(example_linear_least_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_linear_least_square  ${catkin_LIBRARIES})

add_executable(example_least_square_accelerometer_error_parameters_estimation  src/example_least_square_accelerometer_error_parameters_estimation.cpp)
add_dependencies(example_least_square_accelerometer_error_parameters_estimation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_least_square_accelerometer_error_parameters_estimation  ${catkin_LIBRARIES})

add_executable(example_ransac_line_feature_estimate  src/example_ransac_line_feature_estimate.cpp)
add_dependencies(example_ransac_line_feature_estimate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_ransac_line_feature_estimate  ${catkin_LIBRARIES})

add_executable(example_eigen_transformations  src/example_eigen_transformations.cpp)
add_dependencies(example_eigen_transformations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_transformations  ${catkin_LIBRARIES})

add_executable(example_iterative_closest_points_algorithm  src/example_iterative_closest_points_algorithm.cpp)
add_dependencies(example_iterative_closest_points_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_iterative_closest_points_algorithm  ${catkin_LIBRARIES})

add_executable(example_eigen_basics2  src/example_eigen_basics2.cpp)
add_dependencies(example_eigen_basics2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_basics2  ${catkin_LIBRARIES})

add_executable(example_numerical_integration  src/example_numerical_integration.cpp)
add_dependencies(example_numerical_integration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_numerical_integration  ${catkin_LIBRARIES})

add_executable(example_ray_tracing_3d  src/example_ray_tracing_3d.cpp)
add_dependencies(example_ray_tracing_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_ray_tracing_3d  ${catkin_LIBRARIES})

add_executable(example_state_estimation_recursive_mean_covariance_inverse  src/example_state_estimation_recursive_mean_covariance_inverse.cpp)
add_dependencies(example_state_estimation_recursive_mean_covariance_inverse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_state_estimation_recursive_mean_covariance_inverse  ${catkin_LIBRARIES})

add_executable(example_non_linear_least_squares_gauss_newton  src/example_non_linear_least_squares_gauss_newton.cpp)
add_dependencies(example_non_linear_least_squares_gauss_newton ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_non_linear_least_squares_gauss_newton  ${catkin_LIBRARIES})

add_executable(example_eigen_auto_differentiations  src/example_eigen_auto_differentiations.cpp)
add_dependencies(example_eigen_auto_differentiations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_auto_differentiations  ${catkin_LIBRARIES})

add_executable(example_eigen_auto_differentiation_jacobian  src/example_eigen_auto_differentiation_jacobian.cpp)
add_dependencies(example_eigen_auto_differentiation_jacobian ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_auto_differentiation_jacobian  ${catkin_LIBRARIES})

add_executable(example_gradient_descent_minimization  src/example_gradient_descent_minimization.cpp)
add_dependencies(example_gradient_descent_minimization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_gradient_descent_minimization  ${catkin_LIBRARIES})

add_executable(example_root_finding_methods  src/example_root_finding_methods.cpp)
add_dependencies(example_root_finding_methods ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_root_finding_methods  ${catkin_LIBRARIES})

add_executable(example_non_linear_least_saqures_newton  src/example_non_linear_least_saqures_newton.cpp)
add_dependencies(example_non_linear_least_saqures_newton ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_non_linear_least_saqures_newton  ${catkin_LIBRARIES})

add_executable(example_eigen_lavenberg_marquardt  src/example_eigen_lavenberg_marquardt.cpp)
add_dependencies(example_eigen_lavenberg_marquardt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_lavenberg_marquardt  ${catkin_LIBRARIES})

add_executable(example_eigen_numerical_differentiation  src/example_eigen_numerical_differentiation.cpp)
add_dependencies(example_eigen_numerical_differentiation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_eigen_numerical_differentiation  ${catkin_LIBRARIES})

add_executable(example_node_class  include/MyNodeClass.h  src/MyNodeClass.cpp  src/example_node_class.cpp)
add_dependencies(example_node_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_node_class  ${catkin_LIBRARIES})

add_executable(example_pcl_ros_callback
  src/example_pcl_ros_callback.cpp)
add_dependencies(example_pcl_ros_callback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_ros_callback  ${catkin_LIBRARIES})

add_executable(example_pcl_icp  src/example_pcl_icp.cpp)
add_dependencies(example_pcl_icp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_icp  ${catkin_LIBRARIES})

add_executable(example_binary_search_tree src/example_binary_search_tree.cpp)
add_dependencies(example_binary_search_tree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_binary_search_tree ${catkin_LIBRARIES})

add_executable(example_kdtree_search src/example_kdtree_search.cpp)
add_dependencies(example_kdtree_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_kdtree_search ${catkin_LIBRARIES})

add_executable(example_point_in_triangle src/example_point_in_triangle.cpp)
add_dependencies(example_point_in_triangle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_point_in_triangle ${catkin_LIBRARIES})

add_executable(example_pcl_kdtree src/example_pcl_kdtree.cpp)
add_dependencies(example_pcl_kdtree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_kdtree ${catkin_LIBRARIES} )

add_executable(example_rrt_exploration2d src/example_rrt_exploration2d.cpp)
add_dependencies(example_rrt_exploration2d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_rrt_exploration2d ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(example_pcl_cloud_viewer src/example_pcl_cloud_viewer.cpp)
add_dependencies(example_pcl_cloud_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_cloud_viewer ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(example_pcl_frontiers_exploration src/example_pcl_frontiers_exploration.cpp)
add_dependencies(example_pcl_frontiers_exploration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_frontiers_exploration ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(example_octomap_load_file src/example_octomap_load_file.cpp)
add_dependencies(example_octomap_load_file ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_octomap_load_file ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(example_octomap_service_request src/example_octomap_service_request.cpp)
add_dependencies(example_octomap_service_request ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_octomap_service_request ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(example_octomap_create src/example_octomap_create.cpp)
add_dependencies(example_octomap_create ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_octomap_create ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(subscribe_odom_publish_posestamped src/subscribe_odom_publish_posestamped.cpp)
add_dependencies(subscribe_odom_publish_posestamped ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscribe_odom_publish_posestamped ${catkin_LIBRARIES})

add_executable(example_pcl_create_pointcloud2_msg src/example_pcl_create_pointcloud2_msg.cpp)
add_dependencies(example_pcl_create_pointcloud2_msg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_pcl_create_pointcloud2_msg ${catkin_LIBRARIES})

add_executable(example_navigation_send_goal src/example_navigation_send_goal.cpp)
add_dependencies(example_navigation_send_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_navigation_send_goal ${catkin_LIBRARIES})

add_executable(example_navigation_move_base_simple src/example_navigation_move_base_simple.cpp)
add_dependencies(example_navigation_move_base_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_navigation_move_base_simple ${catkin_LIBRARIES})

add_executable(example_navigation_make_plan src/example_navigation_make_plan.cpp)
add_dependencies(example_navigation_make_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_navigation_make_plan ${catkin_LIBRARIES})

add_executable(example_gps_conversions src/example_gps_conversions.cpp)
add_dependencies(example_gps_conversions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_gps_conversions ${catkin_LIBRARIES})

add_executable(example_trilateration_range_only_measurements src/example_trilateration_range_only_measurements.cpp)
add_dependencies(example_trilateration_range_only_measurements ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_trilateration_range_only_measurements ${catkin_LIBRARIES})

add_executable(example_rosparam_list src/example_rosparam_list.cpp)
add_dependencies(example_rosparam_list ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_rosparam_list ${catkin_LIBRARIES})

add_executable(example_message_multiarray_pub src/example_message_multiarray_pub.cpp)
add_dependencies(example_message_multiarray_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_message_multiarray_pub ${catkin_LIBRARIES})

add_executable(example_message_multiarray_sub src/example_message_multiarray_sub.cpp)
add_dependencies(example_message_multiarray_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_message_multiarray_sub ${catkin_LIBRARIES})

add_executable(example_message_subscriber_template src/example_message_subscriber_template.cpp)
add_dependencies(example_message_subscriber_template ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_message_subscriber_template ${catkin_LIBRARIES})

add_executable(example_message_timeout src/example_message_timeout.cpp)
add_dependencies(example_message_timeout ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_message_timeout ${catkin_LIBRARIES})

add_executable(example_boost_state_machine src/example_boost_state_machine.cpp)
add_dependencies(example_boost_state_machine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_boost_state_machine ${catkin_LIBRARIES})

add_executable(example_message_filter src/example_message_filter.cpp)
add_dependencies(example_message_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_message_filter ${catkin_LIBRARIES})

add_executable(example_octree_intersection src/example_octree_intersection.cpp)
add_dependencies(example_octree_intersection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_octree_intersection ${catkin_LIBRARIES})

add_executable(example_filters src/example_filters.cpp)
add_dependencies(example_filters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_filters ${catkin_LIBRARIES})

