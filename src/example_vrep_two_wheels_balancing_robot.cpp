/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

ros::Publisher pub;
double kp = 200, ki = 20, kd = 20;
double sumError = 0, prvError = 0;
double angle = 0;
ros::Time prvTime, currTime;

void callbackGyro(const geometry_msgs::Vector3ConstPtr &msg) {
  currTime = ros::Time::now();
  double dt = (currTime - prvTime).toSec();
  prvTime = currTime;
  angle = angle + msg->x * dt;
  double error = 0 - angle;
  sumError = sumError + error;
  double control = kp * error + ki * sumError + kd * (error - prvError);
  prvError = error;
  geometry_msgs::Twist cmd;
  cmd.linear.x = -control;
  cmd.linear.y = -control;
  cmd.linear.z = dt;
  cmd.angular.x = angle;
  pub.publish(cmd);
  ROS_INFO("gx:%+06.4lf gy:%+06.4f gz:%+06.4f angleX:%+06.4f control:%+06.4f",
           msg->x, msg->y, msg->z, angle, control);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vrep_two_wheel_balancing_robot");
  ros::NodeHandle nh;
  prvTime = ros::Time::now();
  ros::Subscriber sub = nh.subscribe("/gyro", 10, callbackGyro);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  if (argc < 4) {
    ROS_INFO("vrep_two_wheel_balancing_robot kp ki kd");
    return -1;
  } else {
    kp = atof(argv[1]);
    ki = atof(argv[2]);
    kd = atof(argv[3]);
    ROS_INFO("vrep_two_wheel_balancing_robot kp=%f ki=%f kd=%f", kp, ki, kd);
  }

  ros::spin();

  return 0;
}
