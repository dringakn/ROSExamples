/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *    Notes: add actionlib_msgs and actionlib package in CMakeLists
 **/
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <ros_examples/delayactionAction.h>

typedef actionlib::SimpleActionClient<ros_examples::delayactionAction> Client;

void success_callback(const actionlib::SimpleClientGoalState& state,
                      const ros_examples::delayactionResultConstPtr& result) {
  ROS_INFO("Goal success: %s %s %ld", state.toString().c_str(),
           state.getText().c_str(), result->b);
}

void feedback_callback(
    const ros_examples::delayactionFeedbackConstPtr& feedback) {
  ROS_INFO("Feedback:%ld", feedback->c);
}

void active_callback() { ROS_INFO("Goal becomes active"); }

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_action_client_simple");
  ros::NodeHandle nh;
  Client client("delayaction", true);
  client.waitForServer();
  ros_examples::delayactionGoal goal;
  goal.a = 10;
  client.sendGoal(goal, &success_callback, &active_callback,
                  &feedback_callback);
  ros::spin();
  return 0;
}