#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

struct Data
{
  float x, y, z, f1, f2, f3;
  Data()
  {
    x = y = z = f1 = f2 = f3 = 0;
  }
};

inline double euclideanDistanceSq(Data& v1, Data& v2)
{
  return pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2);
}

unsigned int countNearest(vector<Data>& vertices, Data& pt, double dist)
{
  unsigned int count = 0;
  dist = pow(dist, 2);
  for (auto&& v : vertices)
    if (euclideanDistanceSq(v, pt) <= dist)
      count++;
  return count;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_sensor_msg_pointcloud2");
  ros::NodeHandle nh;
  ros::Publisher pub;
  pub = nh.advertise<sensor_msgs::PointCloud2>("/pc2", 1, true);
  sensor_msgs::PointCloud2 msg;
  random_numbers::RandomNumberGenerator rng;

  vector<Data> lst(100, Data());
  float min = -10, max = 10;
  for (auto&& d : lst)
  {
    d.x = rng.uniformReal(min, max);
    d.y = rng.uniformReal(min, max);
    d.z = 0;
  }

  double distCtr = 0, frontierCtr = 0;  // Counters for normalization [0,1]
  Data pt;
  for (unsigned int idx = 0; idx < lst.size(); idx++)
  {
    lst[idx].f1 = euclideanDistanceSq(lst[idx], pt);
    distCtr += lst[idx].f1;
    lst[idx].f2 = countNearest(lst, lst[idx], 1);
    frontierCtr += lst[idx].f2;
  }

  double maxCost = DBL_MIN;
  Data result;
  for (unsigned int idx = 0; idx < lst.size(); idx++)
  {
    // select one with maximum nearest frontiers with minimum distance
    lst[idx].f1 /= frontierCtr;
    lst[idx].f2 /= distCtr;
    lst[idx].f3 = lst[idx].f1 - lst[idx].f2;
    if (lst[idx].f3 > maxCost)
    {
      maxCost = lst[idx].f3;
      result = lst[idx];
    }
  }

  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.height = 1;

  // Create fields for each point
  sensor_msgs::PointField field;
  field.count = 1;
  msg.point_step = 0;

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.name = "x";
  field.offset = 0;
  msg.fields.push_back(field);
  msg.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.name = "y";
  field.offset += sizeof(float);
  msg.fields.push_back(field);
  msg.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.name = "z";
  field.offset += sizeof(float);
  msg.fields.push_back(field);
  msg.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.name = "f1";
  field.offset += sizeof(float);
  msg.fields.push_back(field);
  msg.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.name = "f2";
  field.offset += sizeof(float);
  msg.fields.push_back(field);
  msg.point_step += sizeof(float);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.name = "f3";
  field.offset += sizeof(float);
  msg.fields.push_back(field);
  msg.point_step += sizeof(float);

  msg.width = lst.size();
  msg.row_step = msg.point_step * msg.width;

  // copy data
  msg.data.resize(msg.row_step);
  for (unsigned int i = 0; i < msg.width; i++)
  {
    memcpy(&msg.data[i * msg.point_step + msg.fields[0].offset], &lst[i].x, sizeof(lst[i].x));    // x
    memcpy(&msg.data[i * msg.point_step + msg.fields[1].offset], &lst[i].y, sizeof(lst[i].y));    // y
    memcpy(&msg.data[i * msg.point_step + msg.fields[2].offset], &lst[i].z, sizeof(lst[i].z));    // z
    memcpy(&msg.data[i * msg.point_step + msg.fields[3].offset], &lst[i].f1, sizeof(lst[i].f1));  // f1
    memcpy(&msg.data[i * msg.point_step + msg.fields[4].offset], &lst[i].f2, sizeof(lst[i].f2));  // f2
    memcpy(&msg.data[i * msg.point_step + msg.fields[5].offset], &lst[i].f3, sizeof(lst[i].f3));  // f3
  }

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}