/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      Send a navigation command action to the robot.
      Use RViz command prompt the specify the goal position.
      roslaunch ros_examples simulation.launch
      roslaunch ros_examples navigation_stack.launch
      rosrun ros_examples example_navigation_send_goal -5 0 0

    Notes:
        move_base_simple/goal: (Topic, geometry_msgs/PoseStamped)
        Provides a non-action interface to move_base for users
        that don't care about tracking the execution status of their goals.

        move_base/feedback: (Topic, move_base_msgs/MoveBaseActionFeedback)
        Feedback contains the current position of the base in the world.

        move_base/status: (Topic, actionlib_msgs/GoalStatusArray)
        Provides status information on the goals that are sent to the
        move_base action.

        move_base/result: (Topic, move_base_msgs/MoveBaseActionResult)
        Result is empty for the move_base action.

        make_plan: (Service, nav_msgs/GetPlan)
        Allows an external user to ask for a plan to a given pose
        from move_base without causing move_base to execute that plan.
*/

#include <actionlib/client/simple_action_client.h>  // Action Client
#include <geometry_msgs/Pose.h>                     // Robot pose provided by move_base
#include <move_base_msgs/MoveBaseAction.h>          // Move base action
#include <move_base_msgs/MoveBaseFeedback.h>        // Move base feedback
#include <move_base_msgs/MoveBaseResult.h>          // Move base result
#include <ros/ros.h>

using namespace geometry_msgs;

Pose robPose;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void done_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Goal Done.");
  std::cout << "State: " << state.toString() << std::endl;
  std::cout << "Status: " << state.getText() << std::endl;
  std::cout << "isDone: " << state.isDone() << std::endl;
  std::cout << "result: " << *result << std::endl;
  ros::shutdown();
}

void active_cb()
{
  ROS_INFO("Goal becomes active.");
}

void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
  ROS_INFO("Feedback recieved.");
  // std::cout << fb->base_position.header << std::endl;
  // std::cout << fb->base_position.pose << std::endl;
  robPose = fb->base_position.pose;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_navigation_send_goal");
  ros::NodeHandle nh("~");

  std::string frame_id;
  nh.param<std::string>("frame_id", frame_id, "map");
  double x, y, z;
  if (!nh.param<double>("x", x, 0))
    x = (argc > 1) ? atof(argv[1]) : 0;
  if (!nh.param<double>("y", y, 0))
    y = (argc > 2) ? atof(argv[2]) : 0;
  if (!nh.param<double>("z", z, 0))
    z = (argc > 3) ? atof(argv[3]) : 0;

  // Instinate Move base action client object
  MoveBaseClient client("move_base", true);

  // Wait for the server-client to establish connection
  // 0 = Inf timeout, returns false on timeout
  while (!client.waitForServer(ros::Duration(0)))
    ROS_INFO_STREAM(ros::this_node::getName() << " waiting for the server connection...");
  ROS_INFO_STREAM(ros::this_node::getName() << ", Server connection established.");

  // Create a goal message
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO_STREAM(goal);

  // Send goal to the client if there is a connection between server/client
  if (client.isServerConnected())
  {
    ROS_INFO("Server is connected.");
    // Cancel all goals currently running on the server, if any
    // The goal id is reset: status.id {node_name-goalID-time_stamp}
    // ROS_INFO("Cancelling all previous goals.");
    // client.cancelAllGoals();
    // Send the goal and register the callbacks
    client.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
    ROS_INFO("New goal sent.");
  }
  else
  {
    ROS_INFO("Server is not connected.");
  }

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ROS_INFO("isDone: %d", client.getState().isDone());
    ROS_INFO("getText: %s", client.getState().getText().c_str());
    ROS_INFO("getState: %d", client.getState().state_);
    ROS_INFO("getStateString: %s", client.getState().toString().c_str());
    ROS_INFO_STREAM(robPose);
    ROS_INFO("");

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}