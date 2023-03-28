#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <bits/stdc++.h>
#include <boost/thread.hpp>

volatile int total_calls = 0, triggers = 0;
volatile bool busy = false;

std::mutex mtx;
int max_time_msec = 100;

bool service_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    boost::thread myThread([&]()
                           {
        if (mtx.try_lock())
        {
            busy = true;
            for (int i = 0; i < max_time_msec; i++)
            {
                ros::Duration(0.001).sleep();
            }
            busy = false;
            triggers++;
            mtx.unlock();
        } });

    total_calls++;
    res.success = busy;
    res.message = std::to_string(total_calls) + ", " + std::to_string(triggers);
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "service_server");
    ros::NodeHandle nh;
    ros::ServiceServer server;
    server = nh.advertiseService("service1", service_callback);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}