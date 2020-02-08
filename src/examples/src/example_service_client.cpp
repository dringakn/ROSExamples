/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include "examples/mysrv.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_client");
  if (argc < 2) {
    ROS_WARN("usage: my_client mode");
    return -1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<examples::mysrv>("myservice", false);
  examples::mysrv srv;
  srv.request.id = atoll(argv[1]);

  ros::service::waitForService("myservice",
                               ros::Duration(1)); // -1 to wait for infinity

  if (client.call(srv)) {
    ROS_INFO("Data recieved: %f", srv.response.data);
  } else {
    ROS_ERROR("Failed to call myservice");
    return -1;
  }
  return 0;
}