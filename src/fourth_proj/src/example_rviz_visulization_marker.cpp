#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_visulization_marker");
  ros::NodeHandle nh;
  ros::Publisher pub =
      nh.advertise<visualization_msgs::Marker>("LandMarks", 1, true);
  ros::Rate rate(1);
  visualization_msgs::Marker lm;
  lm.header.frame_id = "map";
  lm.header.stamp = ros::Time::now();
  lm.lifetime = ros::Duration(0.5);
  lm.ns = "LM";
  lm.id = 0;
  lm.type = visualization_msgs::Marker::SPHERE_LIST;
  lm.action = visualization_msgs::Marker::ADD;
  lm.scale.x = lm.scale.y = lm.scale.z = 1;
  lm.pose.position.x = lm.pose.position.y = lm.pose.position.z = 0;
  lm.pose.orientation.x = lm.pose.orientation.y = lm.pose.orientation.z = 0;
  lm.pose.orientation.w = 1;
  lm.color.b = lm.color.g = 0;
  lm.color.r = lm.color.a = 0.5;
  lm.points.resize(6); // For list of items use points
  lm.points[0].x = lm.points[0].y = lm.points[0].z = 0;
  lm.points[1].x = lm.points[1].y = lm.points[1].z = 2;
  lm.points[2].x = lm.points[2].y = lm.points[2].z = 4;
  lm.points[3].x = lm.points[3].y = lm.points[3].z = 6;
  lm.points[4].x = lm.points[4].y = lm.points[4].z = 8;
  lm.points[5].x = lm.points[5].y = lm.points[5].z = 10;
  lm.colors.resize(6); // If not equal to points size then uses the base color
  lm.colors[0].r = lm.colors[0].g = lm.colors[0].b = lm.colors[0].a = 0.25;
  lm.colors[1].r = lm.colors[1].g = lm.colors[1].b = lm.colors[1].a = 0.5;
  lm.colors[2].r = lm.colors[2].g = lm.colors[2].b = lm.colors[2].a = 0.6;
  lm.colors[3].r = lm.colors[3].g = lm.colors[3].b = lm.colors[3].a = 0.8;
  lm.colors[4].r = lm.colors[4].g = lm.colors[4].b = lm.colors[4].a = 0.9;
  lm.colors[5].r = lm.colors[5].g = lm.colors[5].b = lm.colors[5].a = 1.0;
  while (ros::ok()) {
    pub.publish(lm);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
