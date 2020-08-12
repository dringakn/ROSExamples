/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Determine optimized parameters using N measurements and an initial
   parameters
    Pseudocode:
        𝐶𝑜𝑠𝑡_𝑝𝑟𝑒𝑣 = ∞
        𝑓𝑜𝑟 𝑖=2 𝑡𝑜 𝑁_𝑖𝑡𝑟
            𝑋_𝑖=𝑋_(𝑖−1)+(𝐽^𝑇⋅𝐽)^(−1)⋅𝐽^𝑇 𝑅
            𝐶𝑜𝑠𝑡_𝑛𝑒𝑤=1/2⋅𝑅^𝑇⋅𝑅
            if(abs(𝐶𝑜𝑠𝑡_𝑛𝑒𝑤−𝐶𝑜𝑠𝑡_𝑝𝑟𝑒𝑣 )≤eps || 𝐶𝑜𝑠𝑡_𝑛𝑒𝑤≤𝑒𝑝𝑠)
                break;
            𝐶𝑜𝑠𝑡_𝑝𝑟𝑒𝑣=𝐶𝑜𝑠𝑡_𝑛𝑒𝑤
        𝑒𝑛𝑑

        Example:
                𝑁_𝑖𝑡𝑟 = 10
                𝐶𝑜𝑠𝑡_𝑝𝑟𝑒𝑣 = ∞
                𝑓(𝑥,𝑣,𝑘) = (𝑣⋅𝑥)/(𝑘+𝑥)
                𝐽(𝑥,𝑣,𝑘) = [𝑥/(𝑘+𝑥) −(𝑣⋅𝑥)/(𝑘+𝑥)^2]
                𝑋=[0.1 0.1]
                𝑓𝑜𝑟 𝑖=2 𝑡𝑜 𝑁_𝑖𝑡𝑟
                        𝑣=𝑋_(𝑖−1) (1)
                        𝑘=𝑋_(𝑖−1) (2)
                        𝑓𝑜𝑟 𝑗=1 𝑡𝑜 𝑁_𝑑𝑎𝑡𝑎
                                J_𝑓(𝑗,:)=𝐽(𝑋_𝑚 (𝑗),𝑣,𝑘)
                                R_𝑓(𝑗,:)=𝑌_𝑚 (𝑗)−𝑓(𝑋_𝑚 (𝑗),𝑣,𝑘)
                        𝑒𝑛𝑑
                        𝑋_𝑖=𝑋_(𝑖−1)+(𝐽_𝑓^𝑇⋅𝐽_𝑓 )^(−1)⋅𝐽_𝑓^𝑇 𝑅_𝑓
                        𝐶𝑜𝑠𝑡_𝑛𝑒𝑤=1/2⋅𝑅_𝑓^𝑇⋅𝑅_𝑓
                        if(abs(𝐶𝑜𝑠𝑡_𝑛𝑒𝑤−𝐶𝑜𝑠𝑡_𝑝𝑟𝑒𝑣 )≤eps || 𝐶𝑜𝑠𝑡_𝑛𝑒𝑤≤𝑒𝑝𝑠)
                                break;
                        𝐶𝑜𝑠𝑡_𝑝𝑟𝑒𝑣=𝐶𝑜𝑠𝑡_𝑛𝑒𝑤
                𝑒𝑛𝑑

    Notes:
        Array is used Instead of Vector because it support element operations.
        include_directory (... include ${Eigen_INCLUDE_DIRS} ...)

*/

#include <eigen3/Eigen/Dense>
#include <iostream> // cout, ios, endl, std etc
#include <random>   // randome number generator
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_non_linear_least_squares_gauss_newton");
  ros::NodeHandle nh;
  cout.precision(5);
  cout.setf(ios::fixed | ios::showpos);

  std::default_random_engine rng;
  std::normal_distribution<double> dist(0, 0.01);
  auto nrng = [&](double) { return dist(rng); }; // Lambda function

  int n = (argc > 1) ? atoi(argv[1]) : 100;
  double v = (argc > 2) ? atof(argv[2]) : 0.5;
  double k = (argc > 3) ? atof(argv[3]) : 0.75;
  double kg = (argc > 4) ? atof(argv[4]) : 5;
  double vg = (argc > 5) ? atof(argv[5]) : 5;
  int itrs = (argc > 6) ? atoi(argv[6]) : 1000;

  auto model = [&k, &v](double x) { return v * x / (k + x); };
  auto Jacobian = [&k, &v](double x) {
    return RowVector2d(x / (k + x), (-v * x) / pow(k + x, 2));
  };

  RowVectorXd xv = RowVectorXd::LinSpaced(n, 0, 4);
  RowVectorXd yv = xv.unaryExpr(model) + RowVectorXd::NullaryExpr(n, nrng);
  cout << "x:" << xv << endl;
  cout << "y:" << yv << endl;
  double cost, costPrev = DBL_MAX;
  const int N = itrs;      // Maximum number of iterations.
  const double eps = 1e-6; // Minimum change in values
  // !!!!!!!
  // Initial parameters guess, will not converge if initial guess is too far
  // from correct
  Vector2d params(vg, kg);
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
    double error = abs(cost - costPrev);
    cout << i << '\t' << params(0) << '\t' << params(1) << '\t' << cost << '\t'
         << error << endl;
    if (cost <= eps || error <= eps) {
      break; // Exit the iterations if solution converges
    }
    costPrev = cost; // save cost for next iteration
  }

  return 0;
}