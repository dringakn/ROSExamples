/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "random_numbers/random_numbers.h"
#include "ros/ros.h"
#include "ros_examples/mysrv.h"

bool request_callback(ros_examples::mysrv::Request &req,
                      ros_examples::mysrv::Response &res) {
  random_numbers::RandomNumberGenerator rng;
  ROS_INFO("request: id=%ld", (long int)req.id);
  if (req.id == 0) {
    res.data = rng.gaussian(0, 1);
  } else {
    double randomWalk = 0;
    for (int i = 0; i < 1e8; ++i) randomWalk += rng.gaussian(0, 1);
    res.data = randomWalk;
  }
  ROS_INFO("sending back response: [%f]", res.data);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_server");
  ros::NodeHandle nh;

  ros::ServiceServer server =
      nh.advertiseService("myservice", request_callback);
  ROS_INFO("Ready to service");
  ros::spin();

  return 0;
}