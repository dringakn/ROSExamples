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
#include <eigen3/unsupported/Eigen/AutoDiff>
#include <iostream> // cout, ios, endl, std etc
#include <random>   // randome number generator
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

/*
struct F {
  template <typename T> bool operator()(const T *x, const T *y, T *out) {
    out[0] = x[0] * y[0] + x[1] * y[1] - T(10.0);
    return true;
  }
};
*/

// Example function with one parameter f(x) = 2x^2 + 3x + 1
template <typename T> T scalarFunctionOne(T const &x) {
  return 2 * x * x + 3 * x + T(1);
};

// Jacobian df(x)/dx = 4x + 3
void checkFunctionOne(double &x, double &dfdx) { dfdx = 4 * x + 3; }

// Example function with one parameter 𝑓(𝑥,𝑣,𝑘)=(𝑣⋅𝑥)/(𝑘+𝑥)
template <typename T> T scalarFunctionTwo(T const &x, T const &v, T const &k) {
  return (v * x) / (k + x);
};

// 𝑓(𝑥,𝑣,𝑘)=(𝑣⋅𝑥)/(𝑘+𝑥)
// Jacobian [𝑓(𝑥,𝑣,𝑘)/d𝑣 𝑓(𝑥,𝑣,𝑘)/d𝑘]
// 𝐽(𝑥,𝑣,𝑘) = [𝑥/(𝑘+𝑥) −(𝑣⋅𝑥)/(𝑘+𝑥)^2]
void checkFunctionTwo(double &x, double &v, double &k, double &dfdv,
                      double &dfdk) {
  dfdv = x / (k + x);
  dfdk = -(v * x) / pow(k + x, 2);
}

int main(int argc, char **argv) {
  cout.precision(5);
  cout.setf(ios::fixed | ios::showpos);

  cout << endl << "Testing scalar function with 1 input..." << endl;
  Eigen::AutoDiffScalar<Eigen::VectorXd> xA; // Create diff variable
  xA.value() = 1;                            // Evaluation value
  xA.derivatives() =
      Eigen::VectorXd::Unit(1, 0); // Position of variable in the vector
  Eigen::AutoDiffScalar<Eigen::VectorXd> fA;
  fA = scalarFunctionOne(xA); // Differentiation result
  cout << "  AutoDiff:" << endl;
  cout << "    Function output: " << fA.value() << endl; // function evaluation
  cout << "    Derivative: " << fA.derivatives()
       << endl;              // Derivative evaluation
  double x = 1, dfdx;        // varification: evaluation point, derivatie value
  checkFunctionOne(x, dfdx); // Evaluate function and derivative
  cout << "  Hand differentiation:" << endl;
  cout << "    Derivative: " << dfdx << endl << endl;
  cout << "Testing scalar function with 2 inputs..." << endl;

  Eigen::AutoDiffScalar<Eigen::VectorXd> adV, adK, adX, gA;
  adX.value() = 1.0;
  adV.value() = 0.5;
  adK.value() = 0.75;
  adV.derivatives() = Eigen::VectorXd::Unit(2, 0);
  adK.derivatives() = Eigen::VectorXd::Unit(2, 1);
  gA = scalarFunctionTwo(adX, adV, adK);
  cout << "  AutoDiff:" << endl;
  cout << "    Function output: " << gA.value() << endl;
  cout << "    Derivative: " << gA.derivatives()[0] << ", "
       << gA.derivatives()[1] << endl;
  double v = 0.5, k = 0.75, dfdv, dfdk;
  x = 1.0;
  checkFunctionTwo(x, v, k, dfdv, dfdk);
  cout << "  Hand differentiation:" << endl;
  cout << "    Derivative: " << dfdv << ", " << dfdk << endl;

  return 0;
}