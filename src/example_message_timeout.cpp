/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Message timeout example.
        Subscribte to a std_msgs/String on the topic "/test". Start a timer with
        specified duration. If the message arrives, reset the timer, otherwise
        if the timer event occures, set the timeout flag.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace std;

double duration = 1, dT = 0;
ros::Time t1, t2;
bool timeOut = false;
ros::Timer timer;
ros::Subscriber sub;

void stringCallback(const std_msgs::String::ConstPtr& msg) {
  timeOut = false;
  t1 = t2;
  t2 = ros::Time::now();
  dT = (t2 - t1).toSec();
  timer.setPeriod(ros::Duration(duration), true);
  cout << "dT: " << dT << " sec" << endl;
  // cout << *msg << endl;
}

void timerCallback(const ros::TimerEvent& event) {
  timeOut = true;
  cout << "Timeout: " << timeOut << endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_message_timeout");
  ros::NodeHandle nh;
  t1 = t2 = ros::Time::now();
  sub = nh.subscribe<std_msgs::String>("/test", 1, stringCallback);
  timer = nh.createTimer(ros::Duration(duration), &timerCallback, false, true);
  ros::spin();
  return 0;
}