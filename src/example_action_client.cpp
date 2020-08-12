/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
#include "actionlib/client/simple_action_client.h"
#include "ros/ros.h"
#include "ros_examples/myactAction.h"
#include "ros_examples/myactActionGoal.h"
#include "ros_examples/myactFeedback.h"
#include "ros_examples/myactResult.h"

typedef actionlib::SimpleActionClient<ros_examples::myactAction> Client;

void success_callback(const actionlib::SimpleClientGoalState &state,
                      const ros_examples::myactResultConstPtr &result) {
  ROS_INFO("State: %s", state.toString().c_str());
  ROS_INFO("Status: %s", state.getText().c_str());
  ROS_INFO("Result: %f", result->timeElapsed.toSec());
  ros::shutdown();
}

void active_callback() { ROS_INFO("Goal becomes active"); }

void feedback_callback(const ros_examples::myactFeedbackConstPtr &feedback) {
  ROS_INFO("Time remaining: %08.4f", feedback->timeRemaining.toSec());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_client");
  ros::NodeHandle nh;

  if (argc < 2) {
    ROS_INFO("syntax action_client timeToWait");
    return -1;
  }

  Client client(nh, "action_server", true);  // true -> don't need ros::spin()
  client.waitForServer();
  ros_examples::myactGoal goal;
  goal.timeToWait = ros::Duration(atof(argv[1]));
  client.sendGoal(goal, &success_callback, &active_callback,
                  &feedback_callback);
  // ros::Duration(5).sleep();client.cancelGoal(); // rostopic pub
  // /action_server/cancel actionlib_msgs/GoalID
  // client.waitForResult(ros::Duration(5.0));
  ros::spin();
  return 0;
}
