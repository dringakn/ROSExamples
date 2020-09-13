/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Notes:
        https://cosa.th-luebeck.de/files/download/pub/TR-2-2015-least-sqaures-with-ToA.pdf
        cout.ios(ios.fixed|ios.showpos)
        A.completeOrthogonalDecomposition().pseudoInverse()
        vector<vector<double>> d = { { 1, 2, 3, 4 }, { 5, 6, 7, 8 } }
        rosbag filter input.bag output.bag "topic=='/ranges'"
        rosbag filter output.bag output1.bag "topic=='/ranges'" --print="'%f,%f,%f,%f' %
        (m.distances[0],m.distances[1],m.distances[2],m.distances[3])"
*/
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

using namespace std;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Vector3d Vector3;

int main(int argc, char* argv[])
{
  cout.setf(ios::fixed | ios::showpos);
  cout.precision(2);
  ros::init(argc, argv, "example_trilateration_range_only_measurements");
  ros::NodeHandle nh("~");

  // Load the number of anchors
  int anchors;
  if (!nh.param<int>("anchors", anchors, 3))
    nh.setParam("anchors", anchors);
  cout << "Anchors: " << anchors << endl;

  // Load the x,y,z position of each anchor
  vector<vector<double>> anchor(anchors);
  for (int i = 0; i < anchors; i++)
  {
    std::string param = std::string("anchor").append(std::to_string(i + 1));
    if (nh.getParam(param, anchor[i]))
    {
      cout << param << ": ";
      for (auto&& v : anchor[i])
        cout << v << ", ";
      cout << endl;
    }
    else
    {
      cout << "loading parameter '" << param << "' failed." << endl;
    }
  }

  /* Perform the trilateration */
  // Calcualte constants
  vector<double> k(anchors);
  double sum = 0;
  for (auto&& a : anchor)
  {
    sum = 0;
    for (auto&& v : a)
      sum += pow(v, 2.0);
    k.push_back(sum);
  }

  // Calcualte constants using reference anchor
  vector<double> dx, dy, dz, dk;
  for (int i = 1; i < anchors; i++)
  {
    dx.push_back(2 * (anchor[i][0] - anchor[0][0]));
    dy.push_back(2 * (anchor[i][1] - anchor[0][1]));
    dz.push_back(2 * (anchor[i][2] - anchor[0][2]));
    dk.push_back(k[i] - k[0]);
  }

  // Calcualte A matrix
  //   cout.precision(5);
  Matrix3 A;
  A << dx[0], dy[0], dz[0], dx[1], dy[1], dz[1], dx[2], dy[2], dz[2];
  cout << "A:" << endl << A << endl;

  // Determine inverse
  Matrix3 Ainv = A.completeOrthogonalDecomposition().pseudoInverse();
  cout << "Ainv:" << endl << Ainv << endl;

  // Sample range(distance) measuements
  vector<vector<double>> d = {
    { 40.619999, -1.000000, 11.510000, 40.840000 }, { 40.590000, 8.930000, 11.530000, 40.779999 },
    { 40.610001, 8.960000, 11.530000, 40.799999 },  { 40.599998, 8.980000, 11.510000, 40.810001 },
    { 40.639999, 8.950000, 11.540000, 40.790001 },  { 40.580002, 8.950000, 11.520000, 40.770000 },
    { 40.590000, 8.960000, 11.520000, 40.779999 },  { 40.619999, 8.960000, 11.500000, 40.840000 },
    { 40.610001, 8.940000, 11.490000, 40.830002 },  { 40.590000, 8.910000, 11.500000, 40.790001 },
    { 40.570000, 8.960000, 11.520000, 40.799999 },  { 40.549999, 8.920000, 11.510000, 40.790001 },
    { 40.610001, 8.930000, 11.490000, 40.790001 },  { 40.599998, 8.980000, 11.480000, 40.799999 },
    { 40.560001, 8.940000, 11.540000, 40.790001 },  { 40.570000, 8.940000, 11.500000, 40.790001 },
    { 40.580002, 9.000000, 11.530000, 40.799999 },  { 40.599998, 8.980000, 11.530000, 40.790001 },
    { 40.599998, 8.910000, 11.550000, 40.790001 },  { 40.590000, 8.950000, 11.510000, 40.790001 },
    { 40.580002, 8.980000, 11.490000, 40.810001 },  { 40.580002, 8.980000, 11.500000, 40.860001 },
    { 40.599998, -1.000000, -1.000000, 40.77999 },  { 40.560001, 8.930000, 11.530000, 40.759998 },
    { 40.580002, 8.970000, 11.520000, -1.000000 },  { 40.619999, 8.960000, 11.540000, 40.830002 },
    { 40.619999, 8.980000, 11.520000, 40.810001 },  { 40.599998, 9.010000, 11.530000, 40.849998 },
    { 40.570000, 8.940000, 11.490000, 40.840000 },  { 40.619999, 8.910000, 11.490000, 40.759998 },
    { 40.590000, 8.960000, 11.520000, 40.830002 },  { 40.590000, -1.000000, 11.540000, 40.81000 },
    { 40.599998, 8.910000, 11.550000, 40.770000 },  { 40.580002, 8.960000, 11.490000, 40.759998 },
    { 40.590000, 8.930000, 11.480000, 40.759998 },  { 40.570000, 8.950000, 11.500000, 40.799999 },
    { 40.580002, 8.930000, 11.540000, 40.820000 },  { 40.570000, 8.980000, 11.500000, 40.860001 },
    { 40.610001, 8.990000, 11.510000, 40.820000 },  { 40.619999, 8.950000, 11.490000, 40.840000 },
    { 40.619999, 8.970000, 11.520000, 40.810001 },  { 40.610001, 8.960000, 11.520000, 40.799999 },
    { 40.560001, 8.910000, 11.510000, 40.740002 },  { 40.590000, 8.890000, 11.510000, 40.840000 },
    { 40.570000, 8.950000, 11.490000, 40.790001 },  { 40.580002, 8.940000, 11.510000, 40.790001 },
    { 40.590000, 8.940000, 11.520000, 40.779999 },  { 40.570000, 8.950000, 11.490000, 40.810001 },
    { 40.580002, 8.930000, 11.500000, 40.759998 },  { 40.570000, 8.920000, -1.000000, 40.799999 },
    { 40.599998, 8.930000, 11.520000, 40.820000 },  { 40.590000, 8.930000, 11.540000, 40.810001 }
  };

  // Matrix b is created for range measurements and constants

  for (auto&& r : d)
  {
    if (r[0] == -1 or r[1] == -1 or r[2] == -1 or r[3] == -1)
    {
      cout << "Invalid measurement." << endl << endl;
      continue;
    }
    Vector3 b;
    b[0] = r[0] * r[0] - r[1] * r[1] + dk[0];
    b[1] = r[0] * r[0] - r[2] * r[2] + dk[1];
    b[2] = r[0] * r[0] - r[3] * r[3] + dk[2];
    cout << Ainv * b << endl << endl;
  }

  ros::spin();
  return 0;
}