#include "random_numbers/random_numbers.h"
#include "ros/ros.h"

void timerCallback(const ros::TimerEvent &event) {
  ROS_INFO("LE:%f LR:%f CE:%f CR:%f LD:%f", (event.last_expected).toSec(),
           (event.last_real).toSec(), (event.current_expected).toSec(),
           (event.current_real).toSec(),
           (event.profile.last_duration)
               .toSec());     // previous callback execution duration
  ros::Duration(0.1).sleep(); // Wait for 0.1 seconds before exit
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_random_numbers");
  ros::NodeHandle nh;
  ros::Rate rate(1); // Control over loop time
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback, false);
  // timer.start();timer.stop();timer.setPeriod(ros::Duration(1),true);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}