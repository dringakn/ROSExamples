/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/

#include "actionlib/server/simple_action_server.h"
#include "ros/ros.h"
#include "ros_examples/myactAction.h"
#include "ros_examples/myactFeedback.h"
#include "ros_examples/myactGoal.h"
#include "ros_examples/myactResult.h"

typedef actionlib::SimpleActionServer<ros_examples::myactAction> Server;

void goal_callback(const ros_examples::myactGoalConstPtr &goal,
                   Server *actionServer) {
  ros::Time startTime = ros::Time::now();
  ros_examples::myactResult result;
  ros_examples::myactFeedback feedback;
  if (goal->timeToWait.toSec() > 60.0) {
    result.timeElapsed = ros::Duration(ros::Time::now() - startTime);
    actionServer->setAborted(
        result, "Goal aborted because due to greater than 60sec wait!");
    return;
  }
  while ((ros::Time::now() - startTime) < goal->timeToWait) {
    if (actionServer->isPreemptRequested()) {
      result.timeElapsed = ros::Duration(ros::Time::now() - startTime);
      actionServer->setPreempted(result, "Goal preempted");
      return;
    }
    feedback.timeRemaining = goal->timeToWait - (ros::Time::now() - startTime);
    actionServer->publishFeedback(feedback);
    ros::Duration(1).sleep();
  }
  result.timeElapsed = ros::Duration(ros::Time::now() - startTime);
  actionServer->setSucceeded(result, "Goal succeeded");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_server");
  ros::NodeHandle nh;

  Server server(nh, "action_server", boost::bind(&goal_callback, _1, &server),
                false);
  server.start();
  ROS_INFO("Ready to service");
  ros::spin();

  return 0;
}
