/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Determine optimized parameters using N measurements and an initial
    Notes:
        Array is used Instead of Vector because it support element operations.
        include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/AutoDiff>
#include <iostream> // cout, ios, endl, std etc
#include <math.h>
#include <random> // randome number generator
#include <ros/ros.h>

//#define M_PI_4 (3.14152 / 4.0)

using namespace std;
using namespace Eigen;

// typedef chrono::time_point<chrono::steady_clock> time_point;     // Windows
typedef chrono::time_point<chrono::system_clock> time_point; // Linux
time_point get_time() { return chrono::high_resolution_clock::now(); };

template <typename T> struct Robot {
  // Definations required by AutoDiffJacobian
  typedef Matrix<T, 3, 1> InputType;
  typedef Matrix<T, 3, 1> ValueType;
  typedef Matrix<T, ValueType::RowsAtCompileTime, InputType::RowsAtCompileTime> JacobianType;
  // Eigen needs this to determine no. of variables at compile time
  enum {
    InputsAtCompileTime = InputType::RowsAtCompileTime,
    ValuesAtCompileTime = ValueType::RowsAtCompileTime
  };
  size_t inputs() const { return InputsAtCompileTime; }

  // Robot controls (like static variables due to functor)
  T v, phi;
  Robot(T v_, T phi_) : v(v_), phi(phi_) {}

  // Add a functor, robot bicycle model
  template <typename X, typename Y>
  void operator()(const X &state, Y *output) const {
    Y &out = *output;
    const T L = T(1);
    out[0] = v * sin(state[2]) * cos(phi);
    out[1] = v * cos(state[2]) * cos(phi);
    out[2] = v * sin(phi) / L;
  }
};

/** No Jacobian example */
template <typename Scalar> struct Robot2 {
  Robot2() {}
  ~Robot2() {}

  template <typename X, typename U, typename Y>
  void operator()(const X &state, const U &control, Y &output) {
    const Scalar L = Scalar(1);

    output[0] = control[0] * sin(state[2]) * cos(control[1]);
    output[1] = control[0] * cos(state[2]) * cos(control[1]);
    output[2] = control[0] * sin(control[1]) / L;
  }
};

template <typename T, typename Fun> struct Residual {
  T t, dt;
  Fun f;
  Residual(const T &_t = T(1), const T &_dt = T(0.05)) : t(_t), dt(_dt) {}
  template <typename T1, typename T2, typename T3>
  void operator()(const T1 &x0, const T2 &u, T3 &integral) {
    // eigen_assert(u.size()!=2);
    // Initialize the integral
    integral = T3(0);
    T time = T(0);
    T1 x = x0, xdot;
    integral = x0.dot(x0);
    while (time <= t) {
      f(x, u, xdot);
      x +=
          static_cast<T3>(dt) * xdot; // Notice the static_cast, otherwise error
      integral += x.dot(x);
      time += dt;
    }
  }
};

int main(int argc, char **argv) {

  cout.precision(9);

  Vector3d x0(1, 1, M_PI_4), xf, dx(0.1, 0.1, 0.05);
  Vector2d u(1, M_PI_4);
  Matrix3d J;
  Robot<double> rob(10, M_PI_4);
  AutoDiffJacobian<Robot<double>> adJacobian(rob);

  time_point start = get_time();
  for (int i = 0; i < 1000; i++) {
    x0 += dx;
    adJacobian(x0, &xf, &J); // Jacobian is in J
  }
  time_point stop = get_time();
  auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
  cout << "Eigen Time: " << duration.count() * 1.0e-3 << " [mSec]" << endl;
  cout << "Input:" << endl << x0 << endl;
  cout << "Output:" << endl << xf << endl;
  cout << "Jacobian:" << endl << J << endl;

  /*
          Second Example: Residual differentation
  */
  Residual<double, Robot2<double>> res;
  double r;
  x0 << 1, 1, M_PI_4;
  u << 1, M_PI_4;
  res(x0, u, r);
  cout << "Numerical Integration:" << endl;
  cout << "Res:" << r << endl;
  using ADScalar = AutoDiffScalar<Matrix<double, 5, 1>>;
  using outer_deriv_type = Matrix<ADScalar, 5, 1>;
  using outerADScalar = AutoDiffScalar<outer_deriv_type>;
  Matrix<outerADScalar, 3, 1> Ax0;
  Matrix<outerADScalar, 2, 1> Au;

  // Initialize values
  for (int i = 0; i < Ax0.SizeAtCompileTime; ++i)
    Ax0(i).value().value() = x0[i];
  for (int i = 0; i < Au.SizeAtCompileTime; ++i)
    Au(i).value().value() = u[i];

  // Initialize derivatives
  int div_size = Ax0.size() + Au.size();
  int derivative_idx = 0;
  for (int i = 0; i < Ax0.size(); ++i) {
    Ax0(i).value().derivatives() =
        Matrix<double, 5, 1>::Unit(div_size, derivative_idx);
    Ax0(i).derivatives() = Matrix<ADScalar, 5, 1>::Unit(div_size, derivative_idx); // cast? double to ADScalar
    // Initialize Hessian matrix to zero
    for (int idx = 0; idx < div_size; idx++)
      Ax0(i).derivatives()(idx).derivatives() = Matrix<double, 5, 1>::Zero();
    derivative_idx++;
  }
  for (int i = 0; i < Au.size(); ++i) {
    Au(i).value().derivatives() =
        Matrix<double, 5, 1>::Unit(div_size, derivative_idx);
    Au(i).derivatives() = Matrix<ADScalar, 5, 1>::Unit(div_size, derivative_idx);  // cast? double to ADScalar
    for (int idx = 0; idx < div_size; idx++)
      Au(i).derivatives()(idx).derivatives() = Matrix<double, 5, 1>::Zero();
    derivative_idx++;
  }

  outerADScalar Ares;
  res(Ax0, Au, Ares);
  std::cout << "AD result: " << Ares.value().value() << "\n";
  std::cout << "AD derivatives: " << Ares.value().derivatives().transpose()
            << "\n";

  // Ax0(0).value() = 100.0;
  // residual(Ax0, Au, Ares);
  // std::cout << "AD derivatives: " << Ares.derivatives().transpose() <<
  "\n";

  start = get_time();
  for (int i = 0; i < 100; ++i) {
    res(Ax0, Au, Ares);
    Ax0(0).value().value() += dx[0];
    Ax0(1).value().value() += dx[1];
    Ax0(2).value().value() += dx[2];
  }
  stop = get_time();
  duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "AD derivatives: " << Ares.value().derivatives().transpose()
            << "\n";
  std::cout << "Eigen Gradient time: "
            << static_cast<double>(duration.count()) * 1e-3 << " [mSec]"
            << "\n";

  /** @note 10x slower than using simple AD

  /** compute the Hessian */
  /** allocate hessian */
  Eigen::Matrix<double, 5, 5> hessian = Eigen::Matrix<double, 5, 5>::Zero();
  for (int i = 0; i < Ares.derivatives().SizeAtCompileTime; ++i) {
    hessian.middleRows(i, 1) = Ares.derivatives()(i).derivatives().transpose();
  }
  std::cout << "Hessian: "
            << "\n"
            << hessian << "\n";

  return 0;
}
