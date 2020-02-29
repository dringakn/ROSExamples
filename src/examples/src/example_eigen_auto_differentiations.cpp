/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Determine optimized parameters using N measurements and an initial
    Notes:
        Array is used Instead of Vector because it support element operations.
        include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/

#include <eigen3/Eigen/Dense>
#include <iostream> // cout, ios, endl, std etc
#include <random>   // randome number generator
#include <ros/ros.h>
#include <unsupported/Eigen/AutoDiff>

using namespace std;
using namespace Eigen;

struct F {
  template <typename T> bool operator()(const T *x, const T *y, T *out) {
    out[0] = x[0] * y[0] + x[1] * y[1] - T(10.0);
    return true;
  }
};

template <typename T> T scalarFunctionOne(T const &x) {
  return 2 * x * x + 3 * x + 1;
};

void checkFunctionOne(double &x, double &dfdx) { dfdx = 4 * x + 3; }

template <typename T> T scalarFunctionTwo(T const &x, T const &y) {
  return 2 * x * x + 3 * x + 3 * x * y * y + 2 * y + 1;
};

void checkFunctionTwo(double &x, double &y, double &dfdx, double &dfdy) {
  dfdx = 4 * x + 3 + 3 * y * y;
  dfdy = 6 * x * y + 2;
}

int main(int argc, char **argv) {
  cout.precision(5);
  cout.setf(ios::fixed | ios::showpos);
  default_random_engine rng;
  normal_distribution<double> dist(0, 0.01);

  int n = (argc > 1) ? atoi(argv[1]) : 100;

  double k = 0.5, v = 0.75;
  auto model = [&k, &v](double x) { return v * x / (k + x); };
  auto Jacobian = [&k, &v](double x) {
    return RowVector2d(x / (k + x), (-v * x) / pow(k + x, 2));
  };
  auto nrng = [&]() { return dist(rng); };

  RowVectorXd xv = RowVectorXd::LinSpaced(n, 0, 4);
  RowVectorXd yv = xv.unaryExpr(model) + RowVectorXd::NullaryExpr(n, nrng);
  cout << "x:" << xv << endl;
  cout << "y:" << yv << endl;
  double cost, costPrev = DBL_MAX;
  const int N = 1000;      // Maximum number of iterations.
  const double eps = 1e-6; // Minimum change in values
  // !!!!!!!
  // Initial parameters guess, will not converge if initial guess is too far
  // from correct
  Vector2d params(5, 5);
  for (int i = 0; i < N; i++) {
    v = params(0);
    k = params(1);
    MatrixXd J = MatrixXd::Zero(n, 2); // Evaluate jacobians
    VectorXd R = VectorXd::Zero(n);    // Calcualte residuals
    for (int j = 0; j < n; j++) {
      R(j) = yv(j) - model(xv(j));
      J.row(j) = Jacobian(xv(j));
    }
    // J.rowwise() = x.array().unaryExpr(Jacobian); // Need optimization
    // R = y - x.unaryExpr(model);
    // cout << J << endl << "---" << endl;
    // cout << R << endl << "---" << endl;
    MatrixXd Jt = J.transpose();
    params += ((Jt * J).inverse() * Jt * R);
    cost = 0.5 * R.transpose() * R;
    cout << i << '\t' << params(0) << '\t' << params(1) << '\t' << cost << endl;
    if (cost <= eps || abs(cost - costPrev) <= eps) {
      break; // Exit the iterations if solution converges
    }
    costPrev = cost; // save cost for next iteration
  }

  double x, y, z, f, g, dfdx, dgdy, dgdz;
  Eigen::AutoDiffScalar<Eigen::VectorXd> xA, yA, zA, fA, gA;

  cout << endl << "Testing scalar function with 1 input..." << endl;

  xA.value() = 1;
  xA.derivatives() = Eigen::VectorXd::Unit(1, 0);

  fA = scalarFunctionOne(xA);

  cout << "  AutoDiff:" << endl;
  cout << "    Function output: " << fA.value() << endl;
  cout << "    Derivative: " << fA.derivatives() << endl;

  x = 1;
  checkFunctionOne(x, dfdx);

  cout << "  Hand differentiation:" << endl;
  cout << "    Derivative: " << dfdx << endl << endl;

  cout << "Testing scalar function with 2 inputs..." << endl;

  yA.value() = 1;
  zA.value() = 2;

  yA.derivatives() = Eigen::VectorXd::Unit(2, 0);
  zA.derivatives() = Eigen::VectorXd::Unit(2, 1);

  gA = scalarFunctionTwo(yA, zA);

  cout << "  AutoDiff:" << endl;
  cout << "    Function output: " << gA.value() << endl;
  cout << "    Derivative: " << gA.derivatives()[0] << ", "
       << gA.derivatives()[1] << endl;

  y = 1;
  z = 2;
  checkFunctionTwo(y, z, dgdy, dgdz);

  cout << "  Hand differentiation:" << endl;
  cout << "    Derivative: " << dgdy << ", " << dgdz << endl;
  return 0;
}
}