/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        The following code implements an RANdom SAmple Consensu (RANSAC)
        algorithm to extract a line which best fits the points.
        It creates two structures, Point and Line2D to store the points
        and the line parameters (slope, intercept). Furthermore the line
        has an auxiliary parameter called Sum Square Error (SSE), which
        holds the overall error of the inliar points. The code also implements
        a linear weighted least square fit for the points.

        It is a non-deterministic iterative algorithm to estimate parameters of
        a mathematical model from a set of observed data which contains outliers
        (data that don’t fit the model).

        This re-sampling technique generates candidate solution by using the
        subset (minimum number of observations) required to estimate the
        underlying model parameters.

        A basic assumption is that the data
        consist of inliers (data whose distribution can be explained by some
        set of model parameters). Furthermore, there exists a procedure
        which can estimate the optimal parameters of the underlying model.

        For example in case of 2D-Line two points are required to estimate
        the line parameters. The algorithm further require three parametes:
        (1) N:The number of iterations to be performed by the algorithm
        (2) T:A threshold value to determine, if a point fits the line
        (3) D:The number of points required to assert that the line fits well
            to the measurements.
        N can be found by the following formula while T and D are choose
        according to the noise present in the point measurements. If there
        is too much noise then increasing T and lowering D can increase
        the chances of finding a model.
        N = log(1-p)/log(1-w^k)
        k = Minimum number of points required for the model (2 for line)
        p = Probability of success, that the algorithm chooses k inliar points
            in some iteration.
        w = Probability of inliars, choosing an inliar point from
            all the points.
        w = number of inliars in the measurements / total number of points

    Notes:
        include_directory (... include ${Eigen_INCLUDE_DIRS} ...)
        find_package (... random_numbers geometry_msgs sensor_msgs ...)

*/
#include <eigen3/Eigen/Dense>              // Eigen vector, matrix, arrays
#include <geometry_msgs/Point32.h>         // Pointcloud points
#include <iostream>                        // std, vector
#include <random_numbers/random_numbers.h> // Random number functionality
#include <ros/ros.h>                       // ROS functionality
#include <sensor_msgs/PointCloud.h>        // Pointcloud message

using namespace std;

random_numbers::RandomNumberGenerator rng;

struct Point {
  double x, y, z, w;
  Point(double _x = 0, double _y = 0, double _z = 0, double _w = 1) {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
  }
};

struct Line2D {
  double slope, intercept, SSE;
  Line2D(double _slope = 0, double _intercept = 0) {
    slope = _slope;
    intercept = _intercept;
    SSE = 0;
  }
};

// Weighted linear least square estiamte for a 2D-line parameters given a set of
// 2D-points
bool lineEstimateWLS(vector<Point> &pts, Line2D &l) {
  int n = pts.size();
  if (n < 2)
    return false;
  double x, y, sx = 0, sx2 = 0, sy = 0, sxy = 0, sw = 0;
  for (size_t i = 0; i < n; i++) {
    x = pts[i].x;
    y = pts[i].y;
    sw += pts[i].w;
    sx += x;
    sx2 += (x * x);
    sy += y;
    sxy += (x * y);
  }
  double det = (sw * sx2) - (sx * sx);
  if (det == 0)
    return false;
  l.intercept = ((sx2 * sy) - (sx * sxy)) / det;
  l.slope = ((sw * sxy) - (sx * sy)) / det;
  l.SSE = 0;
  for (size_t i = 0; i < n; i++)
    l.SSE += pow(pts[i].y - l.slope * pts[i].x - l.intercept, 2);
  return true;
}

// RANSAC implementation
bool lineEstimateRANSAC(vector<Point> &pts, Line2D &bestLine,
                        vector<Point> &inliarPoints, int maxItrs = 2,
                        int minInliars = 2, double inliarThreshold = DBL_MAX) {
  int n = pts.size();
  if (n < 2)
    return false;
  double bestLineThresh = DBL_MAX;
  // Iterate specified number of time
  for (size_t i = 0; i < maxItrs; i++) {
    // Select two uniformly selected points
    Point p1 = pts[rng.uniformInteger(0, n - 1)];
    Point p2 = pts[rng.uniformInteger(0, n - 1)];
    // Estimate the line parameters (slote, intercept) from the selected points
    Line2D l;
    l.slope = (p2.y - p1.y) / (p2.x - p1.x);
    l.intercept = p1.y - (l.slope * p1.x);
    std::vector<Point> inliar;
    // Check the line for all the points
    for (size_t j = 0; j < n; j++) {
      double lineError = pts[j].y - (l.slope * pts[j].x) - l.intercept;
      // Check if the point is an inliar
      if (fabs(lineError) <= inliarThreshold) {
        inliar.push_back(pts[j]);
        // Check if specified number of inliar points are available
        if (inliar.size() >= minInliars) {
          // Estimate the line parameters using LSE
          if (lineEstimateWLS(inliar, l)) {
            // If the sum square error is less then the previous
            // update the threshold
            if (l.SSE < bestLineThresh) {
              bestLineThresh = l.SSE;
              bestLine = l;
              inliarPoints = inliar;
            }
            break;
          } // line found
        }   // minimum inliars found
      }     // point within threshold
    }       // Points
  }         // Iterations

  return (bestLine.SSE != 0) ? true : false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_ransac_line_feature_estimate");
  ros::NodeHandle nh;
  ros::Publisher pub_pc =
      nh.advertise<sensor_msgs::PointCloud>("pointcloud", 1, true);
  ros::Publisher pub_in =
      nh.advertise<sensor_msgs::PointCloud>("pointcloud_inliars", 1, true);

  // Number of points required to determine line parameters
  int k = 2;
  // Total number of points
  int n = (argc > 1) ? atoi(argv[1]) : 100;
  // p = Probability of Sucess in finding k points
  double p = (argc > 2) ? atof(argv[2]) : 0.99;
  // w = Probability of inliars = inliars/total points
  double w = (argc > 3) ? atof(argv[3]) : 0.5;
  // Minimimu number on points in the inliar set
  int minInliars = (argc > 4) ? atoi(argv[4]) : 2;
  // Threshold to add point to the inliar set
  double inliarThreshold = (argc > 5) ? atof(argv[5]) : 0.01;

  int N = ceil(log(1 - p) / log(1 - pow(w, k)));
  cout << "N:" << N << endl;
  std::vector<Point> pts;
  sensor_msgs::PointCloud pc;
  pc.header.frame_id = "map";
  pc.header.stamp = ros::Time::now();
  for (size_t i = 0; i < n; i++) {
    Point pt;
    pt.x = rng.uniformReal(-5, 5);
    pt.y = rng.uniformReal(-5, 5);
    // pt.z = rng.uniformReal(-5, 5);
    geometry_msgs::Point32 p;
    pts.push_back(pt);
    p.x = pt.x;
    p.y = pt.y;
    pc.points.push_back(p);
  }
  pub_pc.publish(pc);
  Line2D line;
  std::vector<Point> inliars;

  if (lineEstimateRANSAC(pts, line, inliars, N, minInliars, inliarThreshold)) {
    ROS_INFO("Line Found");
    cout << "Slope:" << line.slope << ", Intercept:" << line.intercept
         << ", SSE:" << line.SSE << ", inliars:" << inliars.size() << endl;
    sensor_msgs::PointCloud pcIn;
    pcIn.header.frame_id = "map";
    pcIn.header.stamp = ros::Time::now();
    for (size_t i = 0; i < inliars.size(); i++) {
      geometry_msgs::Point32 p;
      p.x = inliars[i].x;
      p.y = inliars[i].y;
      pcIn.points.push_back(p);
    }
    pub_in.publish(pcIn);
  } else {
    ROS_INFO("Line Not Found");
  }

  ros::spin();
  return 0;
}