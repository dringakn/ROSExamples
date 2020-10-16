/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Publishes a multi-dimensional array message.
*/

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_message_multiarray_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  ros::Publisher pub;
  pub = nh.advertise<std_msgs::Int32MultiArray>("/multidim_array", 10, true);

  std_msgs::Int32MultiArray msg;  // Create a multi-dimensional array message
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.data_offset = 0;
  msg.layout.dim[0].label = "Sensors";  // Rows label
  msg.layout.dim[0].size = 7;           // Number of sensors
  msg.layout.dim[0].stride = 7 * 4;     // Note dim[0] stride is just size of the array
  msg.layout.dim[1].label = "Flags";    // Columns label
  msg.layout.dim[1].size = 4;           // Number of flags for each sensor
  msg.layout.dim[1].stride = 4;         // Number of elements in each row
  msg.data.resize(7 * 4);               // Total array elements

  // Populate the array
  int idx = 1;
  for (auto&& d : msg.data)
    d = idx++;

  while (ros::ok())
  {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}