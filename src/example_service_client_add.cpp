/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Desciption: Client for addition request from the server node
 *                (example_service_server_add).
 *                filename: addsrv.srv
 *
 *                float64 a
 *                float64 b
 *                ---
 *                float64 result
 *
 *                modify add_service_files(... addsrv.srv ...) inside
 *                CMakeLists.txt of the package.
 **/
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <ros_examples/addsrv.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_service_client_add");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  random_numbers::RandomNumberGenerator rng;
  ros::ServiceClient client =
      nh.serviceClient<ros_examples::addsrv>("addservice");

  while (ros::ok()) {
    ros_examples::addsrv srv;
    srv.request.a = rng.uniformInteger(0, 99);
    srv.request.b = rng.uniformInteger(0, 99);
    if (client.call(srv)) {
      ROS_INFO("Sum:%f", srv.response.result);
    } else {
      ROS_INFO("Failed to call service");
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}