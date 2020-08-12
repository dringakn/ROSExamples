/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Assuming a 1D accelrometer is rotated 0-90deg under the influence of
   gravity.
    For every degree rotation it's output is measured which is expected to be
    as follows:
    x = x(theta) = g * cos(theta)
    However the actual measurements are subjected to various errors such as bias
    scale factor, sensitivity drift and unkown error. These errors are modeled
    as follows:
    y = x + B + S*x + K*x^2 + + N(0,sigma^2)
    y(theta) = g*cos(theta) + B + S*g*cos(theta) + K*(g*cos(theta))^2 + +
   N(0,sigma^2)
    The coefficient of the above polynomial equations can be found using
   following
    linear least square equation:
    =>
    [B] = |[n       sum(x)      sum(x^2)]    |^-1[sum(y)    ]
    [S] = |[sum(x)  sum(x^2)    sum(x^3)]    |   [sum(x*y)  ]
    [K] = |[sum(x^2)sum(x^3)    sum(x^4)]    |   [sum(x^2*y)]

    Notes:

    Array is used Instead of Vector because it support element operations.

    include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <random>

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;
typedef Eigen::ArrayXd Array;

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_linear_least_square");
  ros::NodeHandle nh;
  int n = (argc > 1) ? atof(argv[1]) : 6;

  std::default_random_engine rng;
  std::normal_distribution<double> dist(0, (argc > 2) ? atof(argv[2]) : 1e-5);
  auto grng = [&](double) { return dist(rng); };

  Array x = Array::LinSpaced(n, 0, 90 * M_PI / 180);
  /*
    Assuming    g = 9.81
                B = 100 ug, S = 50 ppm, K =  10 ug/g^2
    =>          B = 0.000981, S = 0.00005, K = 1.0194e-6
  */
  const double g = 9.81;
  const double B = 100e-6 * g, S = 50 / 1e6, K = 10e-6 / g;
  // Error Model: Y*(theta) - Y(theta)
  Array y = Array::Constant(n, 1, B) + S * x + K * x.pow(2.0) +
            Array::NullaryExpr(n, 1, grng);
  Matrix A = Matrix::Zero(3, 3);
  Vector b = Vector::Zero(3, 1);

  cout << "x:" << endl << x << endl;
  cout << "y:" << endl << y << endl;

  /*
    A(0, 0) = n;
    A(0, 1) = x.sum();
    A(0, 2) = x.pow(2).sum();
    A(1, 0) = A(0, 1);
    A(1, 1) = A(0, 2);
    A(1, 2) = x.pow(3).sum();
    A(2, 0) = A(1, 1);
    A(2, 1) = A(1, 2);
    A(2, 2) = x.pow(4).sum();
    b(0, 0) = y.sum();
    b(1, 0) = (x * y).sum();
    b(2, 0) = (x.pow(2) * y).sum();
  */
  for (size_t i = 0; i < n; i++) {
    double x1 = x[i], y1 = y[i], x2 = x1 * x1, x3 = x2 * x1, x4 = x3 * x1;
    A(0, 1) += x1;
    A(0, 2) += x2;
    A(1, 2) += x3;
    A(2, 2) += x4;
    b(0, 0) += y1;
    b(1, 0) += (x1 * y1);
    b(2, 0) += (x2 * y1);
  }
  A(0, 0) = n;
  A(1, 0) = A(0, 1);
  A(1, 1) = A(0, 2);
  A(2, 0) = A(1, 1);
  A(2, 1) = A(1, 2);

  Vector sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  cout << "A:" << endl << A << endl;
  cout << "b:" << endl << b << endl;
  cout << "Sol: [B,S,K]" << endl << sol << endl;

  return 0;
}