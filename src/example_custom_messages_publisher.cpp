/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description: Publisehs two custom generate messages on "sensor"
 *                 and "command" topics.
 *               sensmsg.msg:
 *                 Header header
 *                 float64 front
 *                 float64 left
 *                 float64 right
 *               cmdmsg.msg:
 *                 Header header
 *                 float64 vl
 *                 float64 vr
 *               modify add_message_files(... cmdmsg.msg sensmsg.msg ...) inside
 *               CMakeLists.txt of the package.
 **/
#include <ros/ros.h>
#include <ros_examples/cmdmsg.h>
#include <ros_examples/sensmsg.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "example_custom_messages_publisher");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  ros::Publisher pub1 = nh.advertise<ros_examples::sensmsg>("sensor", 1000);
  ros::Publisher pub2 = nh.advertise<ros_examples::cmdmsg>("command", 1000);

  while (ros::ok()) {
    ros_examples::sensmsg s;
    ros_examples::cmdmsg c;
    s.front = 1;
    s.left = 3;
    s.right = 4;
    pub1.publish(s);
    c.vl = 2;
    c.vr = 4;
    pub2.publish(c);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}