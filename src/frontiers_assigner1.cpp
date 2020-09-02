#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <nav_msgs/GetPlan.h>
#include <octomap_frontiers/GetFrontiers.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
typedef octomap::point3d Point3D;
typedef octomath::Vector3 Vector3;
typedef octomath::Quaternion Quaternion;
typedef octomath::Pose6D Pose6D;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

string map_frame, robot_frame;

void done_cb(const actionlib::SimpleClientGoalState& state,
             const move_base_msgs::MoveBaseResultConstPtr& result) {
  // Can start a new frontiers search here
  ROS_INFO_STREAM("Goal Done!!! State: " << state.toString()
                                         << ", : " << state.getText());
}

void active_cb() {
  // May start timer or store current pose for stuck robot measurement!!!
  ROS_INFO("Goal becomes active.");
}

void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
  // Can determine stuck robot here
  // ROS_INFO("FB: (%5.2f, %5.2f, %5.2f)", fb->base_position.pose.position.x,
  //          fb->base_position.pose.position.y,
  //          fb->base_position.pose.position.z);
}

bool getFrontiers(ros::ServiceClient& client, geometry_msgs::Point& min,
                  geometry_msgs::Point& max,
                  std::vector<geometry_msgs::Point>& frontiers,
                  float resolution = 1.0) {
  // Check client
  if (client.exists()) {
    // Create response message
    octomap_frontiers::GetFrontiersResponse res;
    // Create request message
    octomap_frontiers::GetFrontiersRequest req;
    req.min = min;
    req.max = max;
    req.sample_res = resolution;
    // Request service
    if (client.call(req, res)) {
      frontiers = res.frontiers;
      // map_frame = res.header.frame_id;
      return true;
    } else {
      // Request failed
      return false;
    }
  } else {
    // Connection not available
    return false;
  }
}

bool sendGoal(MoveBaseClient& client, geometry_msgs::PoseStamped& pose) {
  // Send goal to the client if there is a connection
  if (client.isServerConnected()) {
    // Create move base goal message
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    client.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
    return true;
  } else {
    // No connection
    return false;
  }
}

bool getRobotPose(tf::TransformListener& li, geometry_msgs::PoseStamped& pose) {
  try {
    // 5sec buffer, ros::Time(0) means we want to transform now.
    tf::StampedTransform st;
    li.waitForTransform(map_frame, robot_frame, ros::Time(0), ros::Duration(1));
    li.lookupTransform(map_frame, robot_frame, ros::Time(0), st);
    pose.pose.position.x = st.getOrigin().x();
    pose.pose.position.y = st.getOrigin().y();
    pose.pose.position.z = st.getOrigin().z();
    pose.pose.orientation.x = st.getRotation().x();
    pose.pose.orientation.y = st.getRotation().y();
    pose.pose.orientation.z = st.getRotation().z();
    pose.pose.orientation.w = st.getRotation().w();
    return true;
  } catch (tf::TransformException ex) {
    ROS_INFO("%s", ex.what());
  }
  return false;
}

bool checkPlan(ros::ServiceClient& client, geometry_msgs::PoseStamped& start,
               geometry_msgs::PoseStamped& goal) {
  // Create a request message
  nav_msgs::GetPlanRequest req;
  req.start = start;
  req.goal = goal;
  req.tolerance = 0.125;
  // Create a response message and call
  nav_msgs::GetPlanResponse res;
  if (client.call(req, res)) {
    return (res.plan.poses.size() > 0) ? true : false;
  } else {
    // Planner declined or plan failed
    return false;
  }
}

geometry_msgs::PoseStamped closestFrontier(
    std::vector<geometry_msgs::Point>& frontiers,
    geometry_msgs::PoseStamped& start, int& id) {
  geometry_msgs::PoseStamped frontier;
  id = -1;
  float min = FLT_MAX, dist;
  for (int i = 0; i < frontiers.size(); ++i) {
    dist = pow(frontiers[i].x - start.pose.position.x, 2) +
           pow(frontiers[i].y - start.pose.position.y, 2) +
           pow(frontiers[i].z - start.pose.position.z, 2);
    if (dist < min) {
      min = dist;
      id = i;
    }
  }
  frontier.header.stamp = ros::Time(0);
  frontier.pose.position = frontiers[id];
  frontier.header.frame_id = map_frame;              // frontier is in map frame
  frontier.pose.position.z = start.pose.position.z;  // <<< CAREFUL >>>
  frontier.pose.orientation.w = 1;
  return frontier;
}

void publishFrontiers(ros::Publisher pubFrontiersPC,
                      vector<geometry_msgs::Point>& frontiers) {
  PointCloud pc;
  pc.header.frame_id = map_frame;
  pc.header.stamp = ros::Time(0).toNSec() / 1000UL;
  pc.is_dense = false;
  pc.height = 1;
  pc.width = frontiers.size();
  for (auto&& f : frontiers) {
    Point p(f.x, f.y, f.z);
    pc.points.push_back(p);
  }
  pubFrontiersPC.publish(pc);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "frontiers_assigner");
  ros::NodeHandle nh("~");
  tf::TransformListener listener(ros::Duration(10));

  // Node parameters
  if (!nh.param<string>("robot_frame", robot_frame, "base_link"))
    nh.setParam("robot_frame", robot_frame);
  if (!nh.param<string>("map_frame", map_frame, "map"))
    nh.setParam("map_frame", map_frame);

  double sample_res, minx, miny, minz, maxx, maxy, maxz;
  if (!nh.param<double>("sample_res", sample_res, 1))
    nh.setParam("sample_res", sample_res);
  if (!nh.param<double>("minx", minx, -10)) nh.setParam("minx", minx);
  if (!nh.param<double>("miny", miny, -10)) nh.setParam("miny", miny);
  if (!nh.param<double>("minz", minz, -10)) nh.setParam("minz", minz);
  if (!nh.param<double>("maxx", maxx, 10)) nh.setParam("maxx", maxx);
  if (!nh.param<double>("maxy", maxy, 10)) nh.setParam("maxy", maxy);
  if (!nh.param<double>("maxz", maxz, 10)) nh.setParam("maxz", maxz);

  // Instinate frontier detection client object
  ros::ServiceClient client_fr =
      nh.serviceClient<octomap_frontiers::GetFrontiers>("get_frontiers");

  // Wait for the server to be available the server
  while (!client_fr.waitForExistence(ros::Duration(1)))  // 0 = Inf
    ROS_INFO("Waiting for the frontiers detector service...");

  // Instinate Move base action client object
  MoveBaseClient client_mb("move_base", true);

  // Wait for the server-client_fr to establish connection
  while (!client_mb.waitForServer(ros::Duration(1)))  // 0 = Inf
    ROS_INFO_STREAM(ros::this_node::getName()
                    << " waiting for the move_base server connection...");

  ros::ServiceClient client_pl;
  client_pl = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

  // Wait for the service to become active
  while (!client_pl.waitForExistence(ros::Duration(1))) {
    ROS_INFO("Waiting for planning service...");
  }

  // Create frontiers visulization
  ros::Publisher pubFrontiersPC;
  pubFrontiersPC = nh.advertise<PointCloud>("/frontiers", 1, true);

  geometry_msgs::Point min, max;
  std::vector<geometry_msgs::Point> frontiers;
  min.x = minx;
  min.y = miny;
  min.z = minz;
  max.x = maxx;
  max.y = maxy;
  max.z = maxz;
  bool state = true;
  ros::Rate rate(1);

  while (ros::ok()) {
    // Should check here the move_base robot state.
    if (client_mb.getState().isDone()) {
      if (getFrontiers(client_fr, min, max, frontiers, sample_res)) {
        if (frontiers.size() > 0) {
          geometry_msgs::PoseStamped pose;
          if (getRobotPose(listener, pose)) {
            int id;
            geometry_msgs::PoseStamped goal =
                closestFrontier(frontiers, pose, id);
            pose.header.frame_id = map_frame;
            goal.header.frame_id = map_frame;
            bool planFound = true;
            while (!checkPlan(client_pl, pose, goal)) {
              ROS_INFO(
                  "No plan found, removing (%5.2f, %5.2f, %5.2f), MoveBase: %s",
                  goal.pose.position.x, goal.pose.position.y,
                  goal.pose.position.z,
                  client_mb.getState().toString().c_str());
              frontiers.erase(frontiers.begin() + id);
              if (frontiers.size() == 0) {
                planFound = false;
                break;
              }
              goal = closestFrontier(frontiers, pose, id);
            }
            if (checkPlan(client_pl, pose, goal)) {
              publishFrontiers(pubFrontiersPC, frontiers);
              if (sendGoal(client_mb, goal)) {
                ROS_INFO("Goal Sent: %5.2f, %5.2f, %5.2f", goal.pose.position.x,
                         goal.pose.position.y, goal.pose.position.z);
                if (client_mb.waitForResult(ros::Duration(10))) {
                  ROS_INFO("Goal reached within specified time.");
                } else {
                  ROS_INFO("Goal didn't reached within time!!!");
                }
              } else {
                ROS_INFO_STREAM(
                    "Sending frontier as goal failed by move_base.");
              }
            } else {
              ROS_INFO(
                  "No plan to the frontier found: (%5.2f, %5.2f, %5.2f) "
                  "MoveBase: %s",
                  goal.pose.position.x, goal.pose.position.y,
                  goal.pose.position.z,
                  client_mb.getState().toString().c_str());
            }
          } else {
            ROS_INFO("Robot pose not found.");
          }
        } else {
          ROS_INFO("No frontiers found");
        }
      } else {
        ROS_INFO("Get frontiers request failed.");
      }
    } else {
      ROS_INFO_STREAM(
          "Move base is currently: " << client_mb.getState().toString());
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}