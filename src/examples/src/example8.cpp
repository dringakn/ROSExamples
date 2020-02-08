/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
#include <ros/ros.h>
#include <examples/delayactionAction.h>
#include <actionlib/client/simple_action_client.h>
// add actionlib_msgs and actionlib package in CMakeLists

typedef actionlib::SimpleActionClient<examples::delayactionAction> Client;

void success_callback(const actionlib::SimpleClientGoalState& state,
                      const examples::delayactionResultConstPtr& result){
    ROS_INFO("Goal success: %s %s %ld",state.toString().c_str(), state.getText().c_str(), result->b);
}

void feedback_callback(const examples::delayactionFeedbackConstPtr& feedback){
    ROS_INFO("Feedback:%ld",feedback->c);
}

void active_callback(){
    ROS_INFO("Goal becomes active");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example8");
    ros::NodeHandle nh;
    Client client("delayaction", true);
    client.waitForServer();
    examples::delayactionGoal goal;
    goal.a = 10;
    client.sendGoal(goal, &success_callback, &active_callback, &feedback_callback);
    ros::spin();
    return 0;
}