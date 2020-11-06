/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Modified:  06 November 2020
    Description: Create random lines and visulaize them in RViz.
**/
#include <geometry_msgs/PoseArray.h>        // Array of robot poses
#include <random_numbers/random_numbers.h>  // Uniform/Normal random number generator
#include <ros/ros.h>                        // ROS functionality
#include <visualization_msgs/Marker.h>      // RVIZ visualization marker

ros::Publisher pubLandMarks;  // LandMarks visualization publisher
random_numbers::RandomNumberGenerator rng;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_visulization_marker_line_strip");
  ros::NodeHandle nh;
  // Create and publish the lanmarks location as visulazition message on the
  // topic "Landmarks"
  pubLandMarks = nh.advertise<visualization_msgs::Marker>("LandMarks", 1, true);

  random_numbers::RandomNumberGenerator rng(0);  // Random number generator

  visualization_msgs::Marker msg;

  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  msg.ns = "LINES";
  msg.id = 0;
  msg.lifetime = ros::Duration(-1);
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  msg.color.b = 1;
  msg.color.a = 1;
  msg.pose.orientation.w = 1;
  msg.scale.x = 0.1;  // For LINE_LIST, y and z are not required

  msg.points.resize(2 * 100000);  // Two points per line
  // If the colors size is 0, then it use the global color
  msg.colors.resize(msg.points.size());  // Line color

  for (int i = 0; i < msg.points.size(); i++)
  {
    msg.points[i].x = rng.uniformInteger(-10, 10);
    msg.points[i].y = rng.uniformInteger(-10, 10);
    msg.points[i].z = rng.uniformInteger(-10, 10);
    // Generate random color if size is specified.
    if (msg.colors.size())
    {
      msg.colors[i].a = 1;
      msg.colors[i].r = rng.uniform01();
      msg.colors[i].g = rng.uniform01();
      msg.colors[i].b = rng.uniform01();
    }
  }

  pubLandMarks.publish(msg);

  ros::spin();
  return 0;
}
