#include <actionlib/client/simple_action_client.h>  // Action Client
#include <move_base_msgs/MoveBaseAction.h>          // Move base action
#include <move_base_msgs/MoveBaseFeedback.h>        // Move base feedback
#include <move_base_msgs/MoveBaseResult.h>          // Move base result
#include <ros/ros.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

void done_cb(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("Done.");
  std::cout << "State: " << state.toString() << std::endl;
  std::cout << "Status: " << state.getText() << std::endl;
  std::cout << "isDone: " << state.isDone() << std::endl;
}

void active_cb() { ROS_INFO("Goal becomes active."); }

void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
  ROS_INFO("Feedback recieved.");
  std::cout << fb->base_position.header << std::endl;
  std::cout << fb->base_position.pose << std::endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_navigation_send_goal");
  ros::NodeHandle nh("~");

  // Instinate Move base action client object
  MoveBaseClient client("move_base", true);

  // Wait for the server-client to establish connection
  while (!client.waitForServer(ros::Duration(1))) {  // 0 = Inf
    // False on timeout
    ROS_INFO_STREAM(ros::this_node::getName()
                    << " waiting for the server connection...");
  }
  ROS_INFO_STREAM(ros::this_node::getName()
                  << ", Server connection established.");
  // Create a goal message
  std::string frame_id;
  nh.param<std::string>("frame_id", frame_id, "map");
  double x, y, z;
  nh.param<double>("x", x, 0);
  nh.param<double>("y", y, 0);
  nh.param<double>("z", z, 0);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time(0);
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  // Send goal to the client if there is a connection between server/client
  if (client.isServerConnected()) {
    ROS_INFO("Server is connected.");
    // Cancel all goals currently running on the server, if any
    // ROS_INFO("Cancelling all previous goals.");
    // client.cancelAllGoals();
    // Send the goal and register the callbacks
    client.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
    ROS_INFO("New goal sent.");
  } else {
    ROS_INFO("Server is not connected.");
  }

  ros::spin();
  return 0;
}