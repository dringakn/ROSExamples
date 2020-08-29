/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include <sstream>

void poseMessageCallback(const turtlesim::Pose::ConstPtr &msg) {
  ROS_INFO("X: [%03.1f] Y: [%03.1f] Theta: [%03.1f] V: [%03.1f] W: [%03.1f]",
           msg->x, msg->y, msg->theta, msg->linear_velocity,
           msg->angular_velocity);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talkershalker");
  ros::NodeHandle nh;
  ros::Publisher driver_pub =
      nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, poseMessageCallback);

  if (argc != 4) {
    ROS_ERROR("usage: example1 x y theta");
    return 1;
  } else {
    double x = atof(argv[1]), y = atof(argv[2]), theta = atof(argv[3]);
    ROS_INFO("X: [%03.1f] Y: [%03.1f] Theta: [%03.1f]", x, y, theta);
    std_srvs::Empty rst;
    nh.serviceClient<std_srvs::Empty>("/reset").call(rst);

    ros::ServiceClient client = nh.serviceClient<turtlesim::TeleportAbsolute>(
        "/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    if (client.call(srv)) {
      ROS_INFO("Succssfully initialized");
    } else {
      ROS_ERROR("Failed to call service teleport_absolute");
      return 1;
    }
  }

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear.x = msg.linear.y = msg.linear.z = 0;
    msg.angular.x = msg.angular.y = msg.angular.z = 0;

    for (int i = 0; i < 4; i++) {
      msg.linear.x = 3;
      msg.angular.z = 0;
      ROS_INFO("Forward Command Sent");
      driver_pub.publish(msg);
      ros::spinOnce();
      ros::Duration(1).sleep();

      msg.linear.x = 0;
      msg.angular.z = M_PI_2;
      ROS_INFO("Rotation Command Sent");
      driver_pub.publish(msg);
      ros::spinOnce();
      ros::Duration(1).sleep();
    }

    ros::Duration(5).sleep();
    ros::spinOnce();
  }
}
