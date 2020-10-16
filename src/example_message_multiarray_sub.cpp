/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Subscribes to a multi-dimensional array message.
*/

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  // cout << *msg << endl; // Debugging purpose
  // Get the Multi-dimensional array information
  vector<vector<int>> data(msg->layout.dim[0].size, vector<int>(msg->layout.dim[1].size));
  // data.assign(msg->data.begin(), msg->data.end());  // copy the elements, error
  // std::copy(msg->data.begin(), msg->data.end(), back_inserter(data));  // copy the elements, error

  // Copy the array, Note the stride of the first dimension contains the size
  for (int r = 0; r < msg->layout.dim[0].size; r++)
  {
    for (int c = 0; c < msg->layout.dim[1].size; c++)
    {
      data[r][c] = msg->data[r * msg->layout.dim[1].size + c];
      cout << data[r][c] << ",";
    }
    cout << endl;
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_message_multiarray_sub");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  sub = nh.subscribe<std_msgs::Int32MultiArray>("/multidim_array", 10, arrayCallback);

  ros::spin();
  return 0;
}