/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

typedef Eigen::MatrixXd Matrix;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example12");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  const unsigned int N = 3;
  Matrix arr[N];
  for (unsigned int i = 0; i < N; i++) {
    arr[i] = Matrix::Zero(3, 1);
    arr[i](0, 0) = arr[i](1, 0);
    std::cout << arr[i](0, 0) << std::endl;
  }

  while (ros::ok()) {
    Matrix m = Matrix::Zero(3, 1);
    m << Matrix::Random(3, 1) * 100;
    std::cout << m << std::endl << std::endl;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}