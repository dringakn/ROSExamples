/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description: Custom server to respond addition request from client node
 *    (example_service_client_add)
 **/
#include <ros/ros.h>
#include <ros_examples/addsrv.h>

bool add(ros_examples::addsrv::Request& req,
         ros_examples::addsrv::Response& resp) {
  resp.result = req.a + req.b;
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_service_server_add");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("addservice", add);
  ros::spin();
  return 0;
}