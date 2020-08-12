/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include "eigen3/Eigen/Dense"          // Matrix algebra library
#include "geometry_msgs/PoseStamped.h" // Resultant pose
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"           // IMU message defination
#include "tf/tf.h"                     // Transformation math
#include "visualization_msgs/Marker.h" // RVIZ visualization marker

typedef Eigen::MatrixXd Matrix;
Matrix A(3, 3), C(3, 3);      // Euler angle estimation and measurement models
Matrix Q(3, 3), R(3, 3);      // Model and measurement uncertainity
Matrix X(3, 1), P(3, 3);      // Estimation variables
const double modelVar = 0.02; // Uncertainity in the predicted state
const double measVar = 0.01;  // Uncertainity in sensor measurement

ros::Publisher pubPose;    // Result publisher
ros::Publisher pubPoseTxt; // Resultant text

void publishResult() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "imu";
  pose.header.stamp = ros::Time::now();
  pose.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(X(0, 0), X(1, 0), X(2, 0));
  pubPose.publish(pose);
  visualization_msgs::Marker text;
  text.header = pose.header;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.color.r = text.color.g = text.color.a = 1;
  text.scale.x = text.scale.y = text.scale.z = 0.25;
  std::stringstream ss;
  ss << std::setprecision(2) << X(0, 0) * 57.3 << " ," << X(1, 0) * 57.3 << " ,"
     << X(2, 0) * 57.3;
  text.text = ss.str();
  pubPoseTxt.publish(text);
}

void kalmanFilter_Initialization() {
  A << 1, 0, 0, 0, 1, 0, 0, 0, 1; // System model
  C << 1, 0, 0, 0, 1, 0, 0, 0, 1; // Observatoin model
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
}

void kalmanFilter_Prediction(double gx, double gy, double gz, double dT) {
  double roll = X(0, 0), pitch = X(1, 0), yaw = X(2, 0);
  double sinroll = sin(roll), cosroll = cos(roll);
  double tanpitch = tan(pitch), cospitch = cos(pitch);
  Matrix U = (Matrix(3, 1) << gx * dT, gy * dT, gz * dT).finished();
  Matrix br2er(3, 3);
  br2er(0, 0) = 1;
  br2er(0, 1) = sinroll * tanpitch;
  br2er(0, 2) = cosroll * tanpitch;
  br2er(1, 0) = 0;
  br2er(1, 1) = cosroll;
  br2er(1, 2) = -sinroll;
  br2er(2, 0) = 0;
  br2er(2, 1) = sinroll / cospitch;
  br2er(2, 2) = cosroll / cospitch;
  X = X + br2er * U;
  P = A * P * A.transpose() + Q;
  publishResult();
}

void kalmanFilter_Correction(double ax, double ay, double az, double dT) {
  double roll = -atan(ay / az), pitch = -atan(ax / sqrt(ay * ay + az * az));
  std::cout << roll * 57.3 << " ," << pitch * 57.3 << std::endl;
  Matrix Z =
      (Matrix(3, 1) << ax, ay, az).finished(); // Create measurement vector
  Matrix K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  // X = X + K * ( Z - C * X );
  // P = ( Matrix::Identity ( 3, 3 ) - K * C ) * P;
}

void callback_IMUmeasurements(const sensor_msgs::ImuConstPtr &msg) {
  ros::Time currTime = msg->header.stamp;
  static ros::Time prevTime = currTime;
  double dT = (currTime - prevTime).toSec();
  double gx = msg->angular_velocity.x, gy = msg->angular_velocity.y,
         gz = msg->angular_velocity.z;
  double ax = msg->linear_acceleration.x, ay = msg->linear_acceleration.y,
         az = msg->linear_acceleration.z;
  kalmanFilter_Prediction(gx, gy, gz, dT);
  kalmanFilter_Correction(ax, ay, az, dT);
  prevTime = currTime;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kalman_filter_sensor_fusion_using_vrep_quadrotor_imu");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/imu", 10, callback_IMUmeasurements);
  pubPose = nh.advertise<geometry_msgs::PoseStamped>("resultantPose", 1, true);
  pubPoseTxt =
      nh.advertise<visualization_msgs::Marker>("resultantPoseTxt", 1, true);
  kalmanFilter_Initialization();
  ros::spin();
  return 0;
}
