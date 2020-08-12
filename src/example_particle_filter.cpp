/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Modified:  10 March 2018
    Description:
            The following boiler plate code is used to complete the lab task 5
of the
            EE565/CS5313 course (Mobile Robotics). The code contains the stubs
labelelled
            with TODO for completion.
**/
#include "ros/ros.h"
#include "tf/tf.h"                      // Transformation math
#include "geometry_msgs/PoseArray.h"       // Array of robot poses
#include "geometry_msgs/Twist.h"           // Navigation command message
#include "geometry_msgs/Vector3.h"         // Landmarks position
#include "random_numbers/random_numbers.h" // Uniform/Normal random number generator
#include "std_msgs/Float64MultiArray.h" // Range measurements message
#include "visualization_msgs/Marker.h"  // RVIZ visualization marker

const unsigned int NP = 100;   // Total number of particles
const unsigned int NLM = 6;    // Six landmarks in the enviornment
const double rangeMeasStd = 1; // Range sensor measurement standard deviation
geometry_msgs::Vector3 Map[6]; // Features (LandMark) Map, Cartesian
                               // coordinates(x,y,z) of each landmark
geometry_msgs::PoseArray Particles; // Particles
double W[NP];                       // Particles weight
double ranges[NLM];                 // Measurements for display
geometry_msgs::PoseStamped
    poseEstimate;               // Estimated (weighted average) particle
ros::Publisher pubLandMarks;    // LandMarks visualization publisher
ros::Publisher pubParticles;    // Particles visualization publiser
ros::Publisher pubMeasurements; // Measurements visualization publiser
ros::Publisher pubEstimate;     // Estimated pose publisher
ros::Publisher pubEstimateTxt;  // Estimate pose information
random_numbers::RandomNumberGenerator rng;

/*
    The following function is used to determine the weighted mean of all the
   sampled particles
    representing the estimated pose.
*/
void meanParticle() {
  double sumW = 0, theta = 0;
  poseEstimate.pose.position.x = poseEstimate.pose.position.y =
      poseEstimate.pose.position.z = 0;
  poseEstimate.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  for (unsigned int i = 0; i < NP; i++) {
    poseEstimate.pose.position.x += W[i] * Particles.poses[i].position.x;
    poseEstimate.pose.position.y += W[i] * Particles.poses[i].position.y;
    poseEstimate.pose.position.z += W[i] * Particles.poses[i].position.z;
    theta += W[i] * tf::getYaw(Particles.poses[i].orientation);
    sumW += W[i];
  }
  poseEstimate.pose.position.x /= sumW;
  poseEstimate.pose.position.y /= sumW;
  poseEstimate.pose.position.z /= sumW;
  theta /= sumW;
  poseEstimate.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
}

/*
    The following function is used to publish particle filter information for
   visulization.
*/
void publishParticles() {
  // Publish particles on the topic "Particles"
  // Ground Truth (-1.2792m, 2.225m, 0.3m, 0degree)
  Particles.header.frame_id = "/map";
  Particles.header.stamp = ros::Time::now();
  pubParticles.publish(Particles);
  // Calulate and publish the estimated pose the robot on the topic
  // "EstimatedPose"
  meanParticle();
  poseEstimate.header = Particles.header;
  pubEstimate.publish(poseEstimate);
  // Create and publish the estimated pose as string for RViz display on the
  // topic "EstimatedPoseTxt"
  visualization_msgs::Marker txt;
  txt.header = Particles.header;
  txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  txt.color.r = 1;
  txt.color.a = 1;
  txt.pose = poseEstimate.pose;
  txt.scale.x = txt.scale.y = txt.scale.z = 0.1;
  std::stringstream ss;
  ss << std::setprecision(2) << poseEstimate.pose.position.x << " ,"
     << poseEstimate.pose.position.y << " ,"
     << tf::getYaw(poseEstimate.pose.orientation) * 57.3 << std::endl;
  //     for(int i=0;i<NP;i++)
  //         ss << std::setprecision ( 2 ) << W[i]<< ">>"
  //         <<Particles.poses[i].position.x << " ," <<
  //         Particles.poses[i].position.y<<" ,"<<tf::getYaw (
  //         Particles.poses[i].orientation ) *57.3<<std::endl;
  txt.text = ss.str();
  pubEstimateTxt.publish(txt);
  // Create and publish the lanmarks location as visulazition message on the
  // topic "Landmarks"
  visualization_msgs::Marker distLM;
  distLM.header = Particles.header;
  distLM.type = visualization_msgs::Marker::LINE_LIST;
  distLM.action = visualization_msgs::Marker::ADD;
  distLM.color.b = 1;
  distLM.color.a = 1;
  distLM.pose.orientation.w = 1;
  distLM.scale.x = distLM.scale.y = distLM.scale.z = 0.01;
  distLM.points.resize(12);
  distLM.colors.resize(6);
  distLM.points[0] = distLM.points[2] = distLM.points[4] = distLM.points[6] =
      distLM.points[8] = distLM.points[10] = poseEstimate.pose.position;
  distLM.points[1].x = Map[0].x;
  distLM.points[1].y = Map[0].y;
  distLM.points[1].z = Map[0].z;
  distLM.points[3].x = Map[1].x;
  distLM.points[3].y = Map[1].y;
  distLM.points[3].z = Map[1].z;
  distLM.points[5].x = Map[2].x;
  distLM.points[5].y = Map[2].y;
  distLM.points[5].z = Map[2].z;
  distLM.points[7].x = Map[3].x;
  distLM.points[7].y = Map[3].y;
  distLM.points[7].z = Map[3].z;
  distLM.points[9].x = Map[4].x;
  distLM.points[9].y = Map[4].y;
  distLM.points[9].z = Map[4].z;
  distLM.points[11].x = Map[5].x;
  distLM.points[11].y = Map[5].y;
  distLM.points[11].z = Map[5].z;
  pubMeasurements.publish(distLM);
}

/*
    The following function sets the landmarks location and color settings for
   publishing.
    Furthermore, it initialize randomly the particles.
*/
void particleFilter_Initialization() {
  Map[0].x = 2.5; // BottomLeft
  Map[0].y = 2.5;
  Map[0].z = 0.3;
  Map[1].x = 0; // BottomMiddle
  Map[1].y = 2.5;
  Map[1].z = 0.3;
  Map[2].x = -2.5; // BottomRight
  Map[2].y = 2.5;
  Map[2].z = 0.3;
  Map[3].x = 2.5; // TopLeft
  Map[3].y = -1.05;
  Map[3].z = 0.3;
  Map[4].x = 0; // TopMiddle
  Map[4].y = -1.05;
  Map[4].z = 0.3;
  Map[5].x = -2.5; // TopRight
  Map[5].y = -1.05;
  Map[5].z = 0.3;

  visualization_msgs::Marker landMarks;
  landMarks.header.frame_id = "/map";
  landMarks.header.stamp = ros::Time::now();
  landMarks.type = visualization_msgs::Marker::SPHERE_LIST;
  landMarks.action = visualization_msgs::Marker::ADD;
  landMarks.ns = "LandMarks";
  landMarks.id = 0;
  landMarks.scale.x = landMarks.scale.y = landMarks.scale.z = 0.1;
  landMarks.points.resize(6);
  landMarks.colors.resize(6);
  landMarks.points[0].x = Map[0].x; // Bottom-Left
  landMarks.points[0].y = Map[0].y;
  landMarks.points[0].z = Map[0].z;
  landMarks.colors[0].r = 0; // Blue
  landMarks.colors[0].g = 0;
  landMarks.colors[0].b = 1;
  landMarks.colors[0].a = 1;
  landMarks.points[1].x = Map[1].x; // Bottom-Middle
  landMarks.points[1].y = Map[1].y;
  landMarks.points[1].z = Map[1].z;
  landMarks.colors[1].r = 0; // Green
  landMarks.colors[1].g = 1;
  landMarks.colors[1].b = 0;
  landMarks.colors[1].a = 1;
  landMarks.points[2].x = Map[2].x; // Bottom-Right
  landMarks.points[2].y = Map[2].y;
  landMarks.points[2].z = Map[2].z;
  landMarks.colors[2].r = 1; // Red
  landMarks.colors[2].g = 0;
  landMarks.colors[2].b = 0;
  landMarks.colors[2].a = 1;
  landMarks.points[3].x = Map[3].x; // Top-Left
  landMarks.points[3].y = Map[3].y;
  landMarks.points[3].z = Map[3].z;
  landMarks.colors[3].r = 0; // Cyan
  landMarks.colors[3].g = 1;
  landMarks.colors[3].b = 1;
  landMarks.colors[3].a = 1;
  landMarks.points[4].x = Map[4].x; // Top-Middle
  landMarks.points[4].y = Map[4].y;
  landMarks.points[4].z = Map[4].z;
  landMarks.colors[4].r = 1; // Magenta
  landMarks.colors[4].g = 0;
  landMarks.colors[4].b = 1;
  landMarks.colors[4].a = 1;
  landMarks.points[5].x = Map[5].x; // Top-Right
  landMarks.points[5].y = Map[5].y;
  landMarks.points[5].z = Map[5].z;
  landMarks.colors[5].r = 1; // Yellow
  landMarks.colors[5].g = 1;
  landMarks.colors[5].b = 0;
  landMarks.colors[5].a = 1;
  pubLandMarks.publish(landMarks);

  // Initialize particles (pose) randomly
  Particles.poses.resize(NP);
  for (unsigned int i = 0; i < NP; i++) {
    Particles.poses[i].position.x = rng.uniformReal(-2.5, 2.5);
    Particles.poses[i].position.y = rng.uniformReal(-1.05, 2.5);
    Particles.poses[i].position.z = 0.3;
    Particles.poses[i].orientation =
        tf::createQuaternionMsgFromYaw(rng.uniformReal(0, 2 * M_PI));
    W[i] = 1.0 / NP;
  }
  publishParticles();
}

/*
    The following function implements the prediction step of the particle filter
   implementation.
    The predition step moves each particle with the specified motion (V,W,T)
   information.
*/
void particleFilter_Prediction(double Vx, double Vy, double Wz, double dT) {
  for (unsigned int i = 0; i < NP; i++) {
    double angle = fmod(tf::getYaw(Particles.poses[i].orientation) + Wz * dT,
                        2 * M_PI); // theta_t = theta_t + dthetas
    Particles.poses[i].orientation = tf::createQuaternionMsgFromYaw(angle);
    double dist = sqrt(pow(Vx * dT, 2) + pow(Vy * dT, 2));
    Particles.poses[i].position.x += dist * cos(angle);
    Particles.poses[i].position.y += dist * sin(angle);
    //         Particles.poses[i].position.x += V * dT * cos(angle);
    //         Particles.poses[i].position.y += V * dT * sin(angle);
  }
}

/*
    The following function determine the probability of the random variable 'x'
   from a Gaussian
    distribuation of specified mean and standard deviation (sigma).
*/
double sampleGaussian(double mean, double sigma, double x) {
  double var = pow(sigma, 2.0); // Variance = sigma^2
  return exp(-pow(mean - x, 2.0) / (2.0 * var)) / sqrt(2.0 * M_PI * var);
}

/*
    The following function is used to determine the probability of the sensor
   measurements 'Z'
    for a given particle representing robot pose X. The probability is
   determined by determining
    the expected measurement of the particle.
*/
double measurement_Probability(geometry_msgs::Pose X, std::vector<double> Z) {
  double prob = 1.0, expectedRange;
  for (unsigned int i = 0; i < NLM; i++) {
    expectedRange = sqrt(pow(X.position.x - Map[i].x, 2.0) +
                         pow(X.position.y - Map[i].y, 2.0));
    prob *= sampleGaussian(expectedRange, rangeMeasStd, Z[i]);
  }
  prob = fabs(prob);
  return prob;
}

/*
    This function implements the correction step of the particle filter
   estimation engine.
    The correction step determines the probability of each particle.
   Furthermore, it normalize
    the probability of the particles.
*/
void particleFilter_Correction(std::vector<double> Z) {
  double sumW = 0;
  for (unsigned int i = 0; i < NP; i++) {
    W[i] = measurement_Probability(Particles.poses[i], Z);
    sumW += W[i];
  }
  for (unsigned int i = 0; i < NP; i++) // Normalize particle weights
  {
    W[i] /= sumW;
  }
}

/*
    This is the most important step of the particle filter. It bascially
   randomly chooses the
    particles based upon their weights. Only the particles with higher weights
   will likely to
    be selected (survived) during the sampling process. Furthermore, in order to
   avoid the
    particles depletion problem some random samples are injected to the
   resultant particles set.
*/
void particleFilter_ReSample() {
  geometry_msgs::Pose S[NP]; // Create an empty particles set
  double SW[NP];             // Create an empty weigths set
  unsigned int idx =
      rng.uniformInteger(0, NP - 1); // Generate random sample index
  double beta = 0.0, maxW = W[0];

  for (unsigned int i = 1; i < NP; i++)
    if (W[i] > maxW)
      maxW = W[i];

  for (unsigned int k = 0; k < NP; k++) {
    beta = beta + rng.uniformReal(0, 2 * maxW);
    while (beta > W[idx]) {
      beta -= W[idx];
      idx = (idx + 1) % (NP - 1);
    }
    S[k] = Particles.poses[idx];
    SW[k] = W[idx];
    // std::cout<<idx<<",";
  }
  // std::cout<<std::endl;
  for (unsigned int k = 0; k < 0.98 * NP; k++) {
    Particles.poses[k] = S[k];
    W[k] = SW[k];
  }
  for (unsigned int k = 0.98 * NP; k < NP; k++) {
    Particles.poses[k].position.x = rng.uniformReal(-2.5, 2.5);
    Particles.poses[k].position.y = rng.uniformReal(-1.05, 2.5);
    Particles.poses[k].position.z = 0.3;
    Particles.poses[k].orientation =
        tf::createQuaternionMsgFromYaw(rng.uniformReal(0, 2 * M_PI));
    W[k] = 1.0 / NP;
  }
  double sumW = 0;
  for (unsigned int k = 0; k < NP; k++) {
    sumW += W[k];
  }
  for (unsigned int k = 0; k < NP; k++) {
    W[k] /= sumW;
  }
}

/*
    The following callback function is called when range sensor measurement are
   obtained by
    the robot on the topic "/myRobot/measurements"
*/
void callback_Measurements(const std_msgs::Float64MultiArray::ConstPtr &Z) {
  for (unsigned int i = 0; i < Z->data.size(); i++)
    ranges[i] = Z->data[i];
  particleFilter_Correction(Z->data);
  particleFilter_ReSample();
  publishParticles();
  // ROS_INFO("BL:%05.3f BM:%05.3f BR:%05.3f TL:%05.3f TM:%05.3f TR:%05.3f",
  // Z->data[0], Z->data[1], Z->data[2], Z->data[3], Z->data[4], Z->data[5]);
}

/*
    The following callback function is called when the odometery information is
   obtaind from
    the robot on the topic "/myRobot/odometry"
*/
void callback_Odometry(const geometry_msgs::Twist::ConstPtr &Cmd) {
  ros::Time currTime = ros::Time::now();
  static ros::Time prevTime = currTime;
  double dT = (currTime - prevTime).toSec();
  double Vx = Cmd->linear.x, Vy = Cmd->linear.y, Wz = Cmd->angular.z;
  prevTime = currTime;
  particleFilter_Prediction(Vx, Vy, Wz, dT);
  publishParticles();
  // ROS_INFO("V:%+05.2f W:%+05.2f dT:%0.4f", V, W, dT);
}

/*
    The main function initialize the node, particles and registers the callback
   functions
    for the sensor and odometry measurements. The three main parts of the
   particle filter
    estimation engine are "particleFilter_Prediction",
   "particleFilter_Correction" and
    "particleFilter_ReSample"
*/
int main(int argc, char **argv) {
  // Initialize the node
  ros::init(argc, argv,
            "ros_particle_filter_node_for_localizing_robot_in_a_vrep_maze");
  ros::NodeHandle nh;
  // Subscribe to odometry and range measurements
  ros::Subscriber subMeas =
      nh.subscribe("/myRobot/measurements", 10, callback_Measurements);
  ros::Subscriber subCmds =
      nh.subscribe("/myRobot/odometry", 10, callback_Odometry);
  // Publish topics
  pubLandMarks = nh.advertise<visualization_msgs::Marker>("LandMarks", 1, true);
  pubMeasurements =
      nh.advertise<visualization_msgs::Marker>("Measurements", 1, true);
  pubParticles = nh.advertise<geometry_msgs::PoseArray>("Particles", 1, true);
  pubEstimate =
      nh.advertise<geometry_msgs::PoseStamped>("EstimatedPose", 1, true);
  pubEstimateTxt =
      nh.advertise<visualization_msgs::Marker>("EstimatedPoseTxt", 1, true);
  // Initialize the filter
  particleFilter_Initialization();
  ros::spin();
  return 0;
}
