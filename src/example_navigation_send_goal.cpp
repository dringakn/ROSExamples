#include <actionlib/client/simple_action_client.h>  // Action Client
#include <move_base_msgs/MoveBaseAction.h>          // Move base action
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

void done_cb(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("State: %s", state.toString().c_str());
  ROS_INFO("Status: %s", state.getText().c_str());
  ROS_INFO_STREAM(result);
  ros::shutdown();
}

void active_cb() { ROS_INFO("Goal becomes active"); }

void feedback_cb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& fb) {
  ROS_INFO_STREAM(fb->header);
  ROS_INFO_STREAM(fb->feedback);
  ROS_INFO_STREAM(fb->status);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_navigation_send_goal");
  ros::NodeHandle nh;

  // Instinate Move base action client object
  MoveBaseClient client(nh, "juma", true);

  // Wait for the server-client to establish connection
  if (!client.waitForServer(ros::Duration(1))) {  // 0 = Inf
    // False on timeout
    ROS_INFO_STREAM(ros::this_node::getName()
                    << " waiting for the server connection...");
  }

  // Create a goal message
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time(0);
  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  // Send goal to the client if there is a connection between server/client
  if (client.isServerConnected()) {
    // Cancel all goals currently running on the server, if any
    client.cancelAllGoals();
    // Send the goal and register the callbacks
    client.sendGoal(goal, done_cb, active_cb, feedback_cb);
  }
  return 0;
}