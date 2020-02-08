// Custom server to respond addition request from client node (example6)
// 
#include <ros/ros.h>
#include <fourth_proj/addsrv.h>

bool add(fourth_proj::addsrv::Request& req, 
         fourth_proj::addsrv::Response& resp){
    resp.result = req.a + req.b;
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example7");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("addservice", add);
    ros::spin();
    return 0;
}