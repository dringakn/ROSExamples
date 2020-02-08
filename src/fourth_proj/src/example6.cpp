// Client for addition request from the server node (example7).
//           addsrv.srv
//             float64 a
//             float64 b
//             ---
//             float64 result
//           modify add_service_files(... addsrv.srv ...) inside CMakeLists.txt
//           of the package.
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <fourth_proj/addsrv.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "exampl6");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    random_numbers::RandomNumberGenerator rng;
    ros::ServiceClient client = nh.serviceClient<fourth_proj::addsrv>("addservice");

    while (ros::ok())
    {
        fourth_proj::addsrv srv;
        srv.request.a = rng.uniformInteger(0,99);
        srv.request.b = rng.uniformInteger(0,99);
        if(client.call(srv)){
            ROS_INFO("Sum:%f", srv.response.result);
        }else{
            ROS_INFO("Failed to call service");
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}