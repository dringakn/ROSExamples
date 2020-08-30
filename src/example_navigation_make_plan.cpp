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
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_navigation_make_plan");
  ros::NodeHandle nh("~");

  // Node parameters
  std::string frame_id;
  nh.param<std::string>("frame_id", frame_id, "map");
  double sx, sy, sz, ex, ey, ez;
  nh.param<double>("start_x", sx, 0);
  nh.param<double>("start_y", sy, 0);
  nh.param<double>("start_z", sz, 0);
  nh.param<double>("end_x", ex, 0);
  nh.param<double>("end_y", ey, 0);
  nh.param<double>("end_z", ez, 0);

  // Create a service client
  ros::ServiceClient client;
  client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

  // Wait for the service to become active
  while (!client.waitForExistence(ros::Duration(1))) {
    ROS_INFO("Waiting for planning service...");
  }

  // Create a request message
  nav_msgs::GetPlanRequest req;
  req.start.header.frame_id = req.goal.header.frame_id = frame_id;
  req.start.header.stamp = req.goal.header.stamp = ros::Time(0);
  req.start.pose.position.x = sx;
  req.start.pose.position.y = sy;
  req.start.pose.position.z = sz;
  req.start.pose.orientation.w = 1;
  req.goal.pose.position.x = ex;
  req.goal.pose.position.y = ey;
  req.goal.pose.position.z = ez;
  req.goal.pose.orientation.w = 1;
  req.tolerance = 0.125;

  // Create a response message and call
  nav_msgs::GetPlanResponse res;
  if (client.call(req, res)) {
    ROS_INFO_STREAM(res.plan.header.frame_id << "\t" << res.plan.poses.size());
  } else {
    ROS_INFO("Plan failed.");
  }

  ros::spin();
  return 0;
}