/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    y(t) = a0 + a1*x(t) + a2*x^2(t) + a3*x^3(t) + ...
    Exmample: y(t) = 3x^2(t) + 2*x(t) + 1 + N(0,sigma^2)
    =>
    [a0] = |[n       sum(x)      sum(x^2)]    |^-1[sum(y)    ]
    [a1] = |[sum(x)  sum(x^2)    sum(x^3)]    |   [sum(x*y)  ]
    [a2] = |[sum(x^2)sum(x^3)    sum(x^4)]    |   [sum(x^2*y)]
    .
    .
    .
    [an] = |[sum(x^n)sum(x^(n+1) sum(x^(n+n))]|   [sum(x^n*y)]
    Notes:

    Array is used Instead of Vector because it support element operations.

    include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;
typedef Eigen::ArrayXd Array;

using namespace std;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_linear_least_square");
  ros::NodeHandle nh;
  int n = (argc > 1) ? atof(argv[1]) : 6;

  std::default_random_engine rng;
  std::normal_distribution<double> dist(0, 0.01);
  auto grng = [&](double) { return dist(rng); };

  Array x = Array::LinSpaced(n, 1, n);
  Array y = 3.0 * x.pow(2.0) + 2.0 * x + Array::Constant(n, 1, 1) +
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

  Vector sol1 = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  Vector sol2 = A.colPivHouseholderQr().solve(b);
  Vector sol3 = (A.transpose() * A).ldlt().solve(A.transpose() * b); // issue!

  cout << "A:" << endl << A << endl;
  cout << "b:" << endl << b << endl;
  cout << "Sol1: [a0,a1,a2]" << endl << sol1 << endl;
  cout << "Sol2: [a0,a1,a2]" << endl << sol2 << endl;
  cout << "Sol3: [a0,a1,a2]" << endl << sol3 << endl;

  return 0;
}