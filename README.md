# ROSExamples
ROS Examples for various basic concepts
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
