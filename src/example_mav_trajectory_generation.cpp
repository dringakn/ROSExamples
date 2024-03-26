/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "../../mav_trajectory_generation/mav_trajectory_generation/include/mav_trajectory_generation/polynomial_optimization_linear.h"
#include "../../mav_trajectory_generation/include/mav_trajectory_generation_ros/ros_conversions.h"
#include "../../mav_trajectory_generation/include/mav_trajectory_generation_ros/ros_visualization.h"
#include <nav_msgs/Odometry.h>
#include <iostream>

class ExamplePlanner
{
public:
  ExamplePlanner(ros::NodeHandle& nh)
    : nh_(nh)
    , max_v_(2.0)
    , max_a_(2.0)
    , current_velocity_(Eigen::Vector3d::Zero())
    , current_pose_(Eigen::Affine3d::Identity())
  {
    // Load params
    if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_))
    {
      ROS_WARN("[example_planner] param max_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_))
    {
      ROS_WARN("[example_planner] param max_a not found");
    }

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for Odometry
    sub_odom_ = nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);
  }

  // Callback to get current Pose of UAV
  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose)
  {
    std::cout << "done" << std::endl;
    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current vleocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
  }

  // Method to set maximum speed.
  void setMaxSpeed(double max_v)
  {
    max_v_ = max_v;
  }

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  // Plans a trajectory from the current position to the a goal position and
  // velocity we neglect attitude here for simplicity
  bool planTrajectory(const Eigen::VectorXd& goal_pos, const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory)
  {
    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), end(dimension);

    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives
    // to zero
    start.makeStartOrEnd(current_pose_.translation(), derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity_);

    // add waypoint to list
    vertices.push_back(start);

    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to
    // zero
    end.makeStartOrEnd(goal_pos, derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel);

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
  }

  bool planTrajectory(const Eigen::VectorXd& goal_pos, const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos, const Eigen::VectorXd& start_vel, double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);

  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory)
  {
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2;  // Distance by which to seperate additional markers.
                            // Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
  }

private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;

  ros::NodeHandle& nh_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_;  // m/s
  double max_a_;  // m/s^2
  double max_ang_v_;
  double max_ang_a_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ExamplePlanner planner(n);
  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  Eigen::Vector3d position, velocity;
  position << 0.0, 1.0, 2.0;
  velocity << 0.0, 0.0, 0.0;

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++)
  {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(position, velocity, &trajectory);
  planner.publishTrajectory(trajectory);
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}