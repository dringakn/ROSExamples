# ROSExamples
ROS Examples for various basic concepts. All example files names are self explanatory except the exampleXX.

## Setup:
```bash
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-robot-localization
sudo apt install libtbb-dev

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
git clone https://github.com/dringakn/ROSExamples.git
git clone https://github.com/dringakn/ufomap.git
git clone https://github.com/dringakn/mav_trajectory_generation

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y -r

```

## AStar:
## Dijkstra:
## example_action_client:
## example_action_server:
## example_cmd_line_params:
## example_custom_message_publisher:
## example_custom_message_subscriber:
## example_eigen_basics:
## example_eigen_basics2:
## example_eigen_transformations:
## example_iterative_closest_points_algorithm:
## example_kalman_filter_sensor_fusion:
## example_kalman_filter_state_estimation:
## example_keyboard_input:
## example_laser_scan_fake:
## example_least_square_accelerometer_error_parameters_estimation:
## example_linear_least_square:
## example_numerical_integration:
## example_occupancy_grid_map:
## example_occupancy_grid_map_sample:
## example_occupancy_grid_map_service_request:
## example_opencv_feature_detection:
## example_opencv_optical_flow:
## example_particle_filter:
## example_path_planning_dijkstra:
## example_path_planning_astar:
## example_pcl2_point_cloud:
## example_pid_control:
## example_random_number_generator:
## example_ransac_line_feature_estimate:
## example_resampling_algorithm:
## example_robot_navigation_command:
## example_rviz_visulization_marker:
## example_service_client:
## example_service_server:
## example_sinewave_generator:
## example_structure_from_motion:
## example_tf:
## example_tf_map_world:
## example_timer_callback:
## example_turtlesim_messages_services:
## example_vrep_demo_robot_controller:
## example_vrep_two_wheels_balancing_robot:

## example1: 
Basic ROS C++ node structure.
## example2: 
Publish a Float32 msg on "topic": sends a uniform random number [0-99] every 100 milliseconds.
modify find_package ( ... random_numbers ...) inside CMakeLists.txt of the package.
## example3: 
Publisehs two custom generated messages on "sensor" and "command" topics.
* sensmsg.msg: 
Header header
float64 front
float64 left
float64 right
* cmdmsg.msg:
Header header
float64 vl
float64 vr
* modify add_message_files(... cmdmsg.msg sensmsg.msg ...) inside CMakeLists.txt of the package.
## example4: 
Subscribe to the custom messages "sensor" and "command"
## example5: 
ROS parameter server example
## example6: 
Client for addition request from the server node (example7).
* addsrv.srv
float64 a
float64 b
 '---
float64 result
* modify add_service_files(... addsrv.srv ...) inside CMakeLists.txt of the package.
## example7: 
Custom server to respond addition request from client node (example6)
## example8: 
Action client
## example9: 
Action server
## example10:
## example11: 
Captures an image from the webcam and publish it on a topic.
## example12: 
Eigen matrix example
## example13: 
PointCloud load (*.ply) example
