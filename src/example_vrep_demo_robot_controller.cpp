/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

ros::Publisher pub;

void callbackUltrasonic(const geometry_msgs::Vector3ConstPtr &distUS) {
  geometry_msgs::Twist cmd;
  const float distMax = 1;
  float V = 0, W = 0;
  float left = distUS->x, front = distUS->y,
        right = distUS->z; // TODO: change the order of x,y,z based upon your
                           // message setting
  if (front == 0)
    front = distMax;
  if (left == 0)
    left = distMax;
  if (right == 0)
    right = distMax;
  // TODO: Write your algorithm to generate the robot navigation command
  // (cmd:[V,W]) based upon the ultrasonic
  // 	     measurements i.e. (left, front and right). if you send the
  // following navigation command  (V=+ve, W=0) then the robot
  //       goes straight, if you send (V=0, W=+ve) then the robot shall rotate
  //       in one direction at it's current position.
  V = 3.0 * front;
  W = 10.0 * (left - right);
  cmd.linear.x = V;
  cmd.angular.z = W;
  cmd.linear.y = cmd.linear.z = cmd.angular.x = cmd.angular.y = 0;
  pub.publish(cmd);
  ROS_INFO("L:%04.2f F:%04.2f R:%04.2f V:%04.2f W:%04.2f", left, front, right,
           V, W);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_client_for_a_obstacle_avoidance_robot_in_vrep");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/robotUS", 10, callbackUltrasonic);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::spin();
  return 0;
}
