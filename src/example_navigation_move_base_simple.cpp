/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Note:
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
#include <actionlib_msgs/GoalStatusArray.h>         // move_base/status
#include <geometry_msgs/PoseStamped.h>              // move_base_simple/goal
#include <move_base_msgs/MoveBaseActionFeedback.h>  // move_base/feedback
#include <move_base_msgs/MoveBaseActionResult.h>    // move_base/result
#include <ros/ros.h>

std::string goalStatus(uint id) {
  switch (id) {
    case actionlib_msgs::GoalStatus::PENDING:
      return std::string("PENDING");
    case actionlib_msgs::GoalStatus::ACTIVE:
      return std::string("ACTIVE");
    case actionlib_msgs::GoalStatus::PREEMPTED:
      return std::string("PREEMPTED");
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      return std::string("SUCCEEDED");
    case actionlib_msgs::GoalStatus::ABORTED:
      return std::string("ABORTED");
    case actionlib_msgs::GoalStatus::REJECTED:
      return std::string("REJECTED");
    case actionlib_msgs::GoalStatus::PREEMPTING:
      return std::string("PREEMPTING");
    case actionlib_msgs::GoalStatus::RECALLING:
      return std::string("RECALLING");
    case actionlib_msgs::GoalStatus::RECALLED:
      return std::string("RECALLED");
    case actionlib_msgs::GoalStatus::LOST:
      return std::string("LOST");
    default:
      return std::string("UNKNOWN");
      break;
  }
}

void status_cb(const actionlib_msgs::GoalStatusArrayConstPtr& msg) {
  ROS_INFO_STREAM("Status:" << msg->header.frame_id
                            << ", List Size:" << msg->status_list.size());
  /*
    Status: actionlib_msgs::GoalStatus
        PENDING = 0u,
        ACTIVE = 1u,
        PREEMPTED = 2u,
        SUCCEEDED = 3u,
        ABORTED = 4u,
        REJECTED = 5u,
        PREEMPTING = 6u,
        RECALLING = 7u,
        RECALLED = 8u,
        LOST = 9u,
  */
  for (auto&& m : msg->status_list)
    std::cout << "ID:" << m.goal_id.id << ", Status:" << goalStatus(m.status)
              << ", Text:" << m.text << std::endl;
}

void feedback_cb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& msg) {
  ROS_INFO_STREAM("Feedback:" << msg->header.frame_id);
  std::cout << msg->feedback << std::endl;
  std::cout << "ID:" << msg->status.goal_id.id
            << ", Status:" << goalStatus(msg->status.status)
            << ", Text:" << msg->status.text << std::endl
            << std::endl;
}

void result_cb(const move_base_msgs::MoveBaseActionResultConstPtr& msg) {
  ROS_INFO_STREAM("Result:" << msg->header.frame_id);
  std::cout << "ID:" << msg->status.goal_id.id
            << ", Status:" << goalStatus(msg->status.status)
            << ", Text:" << msg->status.text << std::endl
            << std::endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_navigation_move_base_simple");
  ros::NodeHandle nh("~");

  // Wait for move base to up and running
  ros::Duration(5);

  // Goal publisher
  ros::Publisher pubGoal;
  pubGoal =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  // Goal status
  ros::Subscriber subStatus;
  subStatus = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",
                                                            1, status_cb);

  // Goal Feedback
  ros::Subscriber subFeedback;
  subFeedback = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>(
      "/move_base/feedback", 1, feedback_cb);

  // Result
  ros::Subscriber subResult;
  subResult = nh.subscribe<move_base_msgs::MoveBaseActionResult>(
      "/move_base/result", 1, result_cb);

  std::string frame_id;
  nh.param<std::string>("frame_id", frame_id, "map");
  double x, y, z;
  nh.param<double>("x", x, 0);
  nh.param<double>("y", y, 0);
  nh.param<double>("z", z, 0);

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time(0);
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.w = 1;
  pubGoal.publish(msg);

  ros::spin();
  return 0;
}