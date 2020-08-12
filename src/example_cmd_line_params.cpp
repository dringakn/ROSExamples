/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_command_line_parameters");
  ros::NodeHandle nh;
  double param = 0.0;

  // For debugging
  // for (int i = 0; i < argc; i++)
  //     ROS_INFO("argv: %s", argv[i]);

  if (!nh.hasParam("param")) {
    ROS_INFO(
        "Parameter (param) doesn't exist, creating with default value: 1.0");
    nh.setParam("param", 1.0);
  } else if (argc > 1) {
    param = atof(argv[1]);
    ROS_INFO("Setting param value: %f", param);
    nh.setParam("param", param);
  }

  ros::Rate rate(0.5);
  while (ros::ok()) {
    // rosparam list to display list of parameters
    // rosparam set param 2, to set a parameter value
    nh.getParamCached("param", param);
    ROS_INFO("param: %f", param);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
