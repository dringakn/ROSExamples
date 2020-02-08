#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

typedef Eigen::MatrixXd Matrix;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example12");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    while (ros::ok())
    {
        Matrix m = Matrix::Zero(3,1);
        m << Matrix::Random(3,1)*100;
        std::cout << m << std::endl << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}