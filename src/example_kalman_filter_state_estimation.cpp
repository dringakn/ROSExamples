/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "eigen3/Eigen/Dense" // Matrix Algebra Library
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

typedef Eigen::MatrixXd Matrix;

Matrix A(3, 3), C(3, 3);      // Wall distance estimation and measurement model
Matrix Q(3, 3), R(3, 3);      // Model and measurement uncertainity
Matrix X(3, 1), P(3, 3);      // Estimation variables
const double modelVar = 0.02; // Uncertainity in the predicted state
const double measVar = 0.01;  // Uncertainity in sensor measurement
const double wallDistance = 0.2; // Distance from wall

void kalmanFilter_Prediction(Matrix &X, Matrix &P) {
  X = A * X;
  P = A * P * A.transpose() + Q;
}

void kalmanFilter_Correction(Matrix &X, Matrix &P, Matrix &Z) {
  Matrix K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  X = X + K * (Z - C * X);
  P = (Matrix::Identity(3, 3) - K * C) * P;
}

void callbackUltrasonic(const geometry_msgs::Vector3ConstPtr &distUS) {
  double left = distUS->x, front = distUS->y, right = distUS->z;
  Matrix Z = (Matrix(3, 1) << left, front, right)
                 .finished(); // Create measurement vector
  kalmanFilter_Correction(X, P, Z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv,
            "ros_Kalman_filter_node_for_a_wall_following_robot_in_vrep");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/myRobot/robotUS", 2, callbackUltrasonic);
  ros::Publisher cmdPub =
      nh.advertise<geometry_msgs::Twist>("/myRobot/cmd_vel", 1);
  ros::Publisher estimatePub =
      nh.advertise<geometry_msgs::Vector3>("/myRobot/estimateUS", 1);
  geometry_msgs::Twist cmd;
  geometry_msgs::Vector3 estimate;
  ros::Rate rate(50);

  A << 1, 0, 0, 0, 1, 0, 0, 0, 1; // System model
  C << 1, 0, 0, 0, 1, 0, 0, 0, 1; // Observatoin model
  X << 0, 0, 0;                   // TODO: Initial state value
  P << 0, 0, 0, 0, 0, 0, 0, 0, 0; // TODO: Initial state's' uncertainity
  Q << modelVar, 0, 0, 0, modelVar, 0, 0, 0,
      modelVar; // TODO: Model error in each state
  R << measVar, 0, 0, 0, measVar, 0, 0, 0,
      measVar; // TODO: Measurement noise in each sensor
  std::cout << "X:" << std::endl
            << X << std::endl
            << "P:" << std::endl
            << P << std::endl
            << "Q:" << std::endl
            << Q << std::endl
            << "R:" << std::endl
            << R << std::endl;

  while (ros::ok()) {
    kalmanFilter_Prediction(X, P);
    estimate.x = X(0, 0);
    estimate.y = X(1, 0);
    estimate.z = X(2, 0);
    estimatePub.publish(
        estimate); // Publish the estimated values for visulization

    // TODO: Use the estimate values [X(0,0) X(1,0) X(2,0)] to calculate robot's
    // V and W
    //       to control your robot
    double V = 3 * X(1, 0);
    double W = 400.0 * (X(0, 0) - wallDistance) + 50.0 * (X(1, 0) - 1);

    // Publish the calculated control commands to the robot
    cmd.linear.x = V;
    cmd.angular.z = W;
    cmd.linear.y = cmd.linear.z = cmd.angular.x = cmd.angular.y = 0;
    cmdPub.publish(cmd);
    ros::spinOnce();
    rate.sleep();
  }

  // Send the command to stop the robot if CTRL+C is pressed
  cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x = cmd.angular.y =
      cmd.angular.z = 0;
  cmdPub.publish(cmd);
  return 0;
}
