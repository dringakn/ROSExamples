/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Notes:
      sudo apt-get install ros-kinect-control-toolbox
      modify find_package (... control_toolbox ...) in CMakeLists.txt of the
      package.
*/

#include <control_toolbox/pid.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_pid_controller");
  ros::NodeHandle nh;
  double kp = 1.0, ki = 1.0, kd = 1.0;
  double setpoint = 1, input = 0, output = 0;
  double imax = 10, imin = -imax;
  control_toolbox::Pid pid(kp, ki, kd, imax, imin);  // Enable integral windeup;
  ros::Time currTime = ros::Time::now(), prevTime = currTime;
  ros::Rate rate(1);
  while (ros::ok()) {
    currTime = ros::Time::now();
    output = pid.computeCommand(setpoint - input, currTime - prevTime);
    pid.printValues();
    prevTime = currTime;
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
