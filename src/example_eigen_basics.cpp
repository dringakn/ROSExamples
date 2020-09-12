/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *     Notes:
 *     REFERENCE: https://eigen.tuxfamily.org/dox/GettingStarted.html
 *          https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
 *
 *          Size can be 2,3,4 for fixed size square matrices or X for dynamic size
 **/

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

typedef Eigen::MatrixXd Matrix;

double foo(double x)
{
  return (x > 0) ? 1 : 0;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "example_eigen_basics");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  // Creation
  Eigen::Matrix4d m;                  // 4x4 double
  Eigen::Matrix4cd objMatrix4cd;      // 4x4 double complex
  Eigen::Vector3f objVector3f;        // Vector3f is a fixed column vector of 3 floats:
  Eigen::RowVector2i objRowVector2i;  // RowVector2i is a fixed row vector of int
  Eigen::VectorXf objv(10);           // VectorXf is a column vector of size 10 floats:
  // Initialization
  Eigen::Matrix2d a_2d;
  a_2d.setRandom();
  a_2d.setConstant(4.3);
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(3, 3);
  Eigen::ArrayXXf table(10, 4);
  table.col(0) = Eigen::ArrayXf::LinSpaced(10, 0, 90);
  // Matrix Coefficient Wise Operations
  Eigen::Matrix<double, 2, 3> my_matrix;
  my_matrix << 1, 2, 3, 4, 5, 6;
  // 1 2 3
  // 4 5 6
  int i, j;
  std::cout << my_matrix.minCoeff(&i, &j) << std::endl;  // 1
  std::cout << my_matrix.maxCoeff(&i, &j) << std::endl;  // 6
  std::cout << my_matrix.transpose() << std::endl;
  // 1 4
  // 2 5
  // 3 6
  std::cout << my_matrix.prod() << std::endl;                     // 720
  std::cout << my_matrix.sum() << std::endl;                      // 21
  std::cout << my_matrix.mean() << std::endl;                     // 3.5
  std::cout << my_matrix.trace() << std::endl;                    // 1+5 = 6
  std::cout << my_matrix.colwise().mean() << std::endl;           // 2.5, 3.5, 4.5
  std::cout << my_matrix.rowwise().maxCoeff() << std::endl;       // 3, 6
  std::cout << my_matrix.lpNorm<2>() << std::endl;                // 9.53939
  std::cout << my_matrix.lpNorm<Eigen::Infinity>() << std::endl;  // 6
  std::cout << (my_matrix.array() > 0).all() << std::endl;        // if all elemnts are positive
  std::cout << (my_matrix.array() > 2).any() << std::endl;        // if any element is greater than 2
  std::cout << (my_matrix.array() > 1).count() << std::endl;      // count the number of elements greater than 1
  std::cout << my_matrix.array() - 2 << std::endl;
  std::cout << my_matrix.array().abs() << std::endl;
  std::cout << my_matrix.array().square() << std::endl;
  std::cout << my_matrix.array() * my_matrix.array() << std::endl;
  std::cout << my_matrix.array().exp() << std::endl;
  std::cout << my_matrix.array().log() << std::endl;
  std::cout << my_matrix.array().sqrt() << std::endl;
  // Block Elements Access
  // Block of size (p,q), starting at (i,j)	matrix.block(i,j,p,q)
  Eigen::MatrixXf mat(4, 4);
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  std::cout << "Block in the middle" << std::endl;
  std::cout << mat.block<2, 2>(1, 1) << std::endl;

  for (int i = 1; i <= 3; ++i)
  {
    std::cout << "Block of size " << i << "x" << i << std::endl;
    std::cout << mat.block(0, 0, i, i) << std::endl;
  }
  // Eigen Rotation Matrices
  double roll = M_PI / 3, pitch = M_PI / 4, yaw = M_PI / 6;
  Eigen::AngleAxisd rAng(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pAng(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yAng(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> q = yAng * pAng * rAng;
  Eigen::Matrix3d rmat = q.matrix();
  std::cout << "roll is Pi/" << M_PI / atan2(rmat(2, 1), rmat(2, 2)) << std::endl;
  std::cout << "pitch: Pi/"
            << M_PI / atan2(-rmat(2, 0), std::pow(rmat(2, 1) * rmat(2, 1) + rmat(2, 2) * rmat(2, 2), 0.5)) << std::endl;
  std::cout << "yaw is Pi/" << M_PI / atan2(rmat(1, 0), rmat(0, 0)) << std::endl;

  Eigen::MatrixXd dynamicMatrix;
  int rows, cols;
  rows = 3;
  cols = 2;
  dynamicMatrix.resize(rows, cols);
  dynamicMatrix << -1, 7, 3, 4, 5, 1;

  // If you want a conservative variant of resize() which does not change the
  // coefficients, use conservativeResize()
  dynamicMatrix.conservativeResize(dynamicMatrix.rows(), dynamicMatrix.cols() + 1);
  dynamicMatrix.col(dynamicMatrix.cols() - 1) = Eigen::Vector3d(1, 4, 0);

  dynamicMatrix.conservativeResize(dynamicMatrix.rows(), dynamicMatrix.cols() + 1);
  dynamicMatrix.col(dynamicMatrix.cols() - 1) = Eigen::Vector3d(5, -8, 6);
  /*
      you should expect this:
       1  7  1  5
       3  4  4 -8
       5  1  0  6

  */
  std::cout << dynamicMatrix << std::endl;

  Matrix mat1 = Matrix::Random(3, 3);
  std::cout << "MAT1:" << std::endl << mat1 << std::endl;
  mat1 = mat1.unaryExpr([](double e) { return (e > 0) ? 1.0 : 0.0; });
  std::cout << "MAT1:" << std::endl << mat1 << std::endl;
  std::cout << "MAT1:" << std::endl << mat1.unaryExpr(std::ptr_fun(foo)) << std::endl;

  const unsigned int N = 3;
  Matrix arr[N];
  for (unsigned int i = 0; i < N; i++)
  {
    arr[i] = Matrix::Zero(3, 1);
    arr[i](0, 0) = arr[i](1, 0);
    // std::cout << arr[i](0, 0) << std::endl;
  }

  while (ros::ok())
  {
    Matrix m = Matrix::Zero(3, 1);
    m << Matrix::Random(3, 1) * 100;
    // std::cout << m << std::endl << std::endl;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}