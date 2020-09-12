/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Notes:
*/
#include <bits/stdc++.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_rosparam_list");
  ros::NodeHandle nh("~");

  int lists;
  if (!nh.param<int>("lists", lists, 0))
    nh.setParam("lists", lists);
  vector<vector<double>> list(lists);
  int idx = 1;
  for (auto&& l : list)
  {
    std::string param = std::string("list").append(std::to_string(idx++));
    if (nh.getParam(param, l))
    {
      cout << param << endl;
      for (auto&& v : l)
        cout << v << endl;
    }
    else
    {
      cout << "loading parameter '" << param << "' failed." << endl;
    }
  }

  ros::spin();
  return 0;
}