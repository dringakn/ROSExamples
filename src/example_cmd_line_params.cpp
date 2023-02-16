/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <ros/ros.h>

/**
 * @brief Get the value of the option flag if it exist.
 *  Example: char * filename = getCmdOption(argv, argv + argc, "-f");
 *
 * @param begin start of the list of string array, e.g. argv
 * @param end end of the list of string array, e.g. argv+argc
 * @param option command line switch, e.g. "-h"
 * @return char*
 */
char *getCmdOption(char **begin, char **end, const std::string &option)
{
    char **itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

/**
 * @brief Check if the option flag (switch) exists.
 *  Example: if(cmdOptionExists(argv, argv+argc, "-h")){...}
 *
 * @param begin start of the list of string array, e.g. argv
 * @param end end of the list of string array, e.g. argv+argc
 * @param option command line switch, e.g. "-h"
 * @return true
 * @return false
 */
bool cmdOptionExists(char **begin, char **end, const std::string &option)
{
    return std::find(begin, end, option) != end;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_command_line_parameters");
  ros::NodeHandle nh;
  double param = 0.0;

  // Check if command line option is provided.
  std::string file_name = "";
  if (cmdOptionExists(argv, argv + argc, "-f"))
    file_name = getCmdOption(argv, argv + argc, "-f");
        
  // For debugging
  // for (int i = 0; i < argc; i++)
  //     ROS_INFO("argv: %s", argv[i]);

  if (!nh.hasParam("param")) {
    ROS_INFO(
        "Parameter (param) doesn't exist, creating with default value: 1.0");
    nh.setParam("param", 1.0);
  } else if (argc > 1) {
    param = atof(argv[1]);
    ROS_INFO("Setting param value: %f", param);
    nh.setParam("param", param);
  }

  ros::Rate rate(0.5);
  while (ros::ok()) {
    // rosparam list to display list of parameters
    // rosparam set param 2, to set a parameter value
    nh.getParamCached("param", param);
    ROS_INFO("param: %f", param);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
