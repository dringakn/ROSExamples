/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
// Custom server to respond addition request from client node (example6)
//
#include <ros/ros.h>
#include <ros_examples/addsrv.h>

bool add(ros_examples::addsrv::Request& req,
         ros_examples::addsrv::Response& resp) {
  resp.result = req.a + req.b;
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example7");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("addservice", add);
  ros::spin();
  return 0;
}