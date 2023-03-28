#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Empty.h>

ros::ServiceClient client;

void subscriber1_callback(const std_msgs::Empty::ConstPtr &msg)
{
    std_srvs::Trigger s;
    if (client.call(s))
    {
        ROS_INFO("Service call successfull: %d, %s", s.response.success, s.response.message.c_str());
    }
    else
    {
        ROS_INFO("Service call failed.");
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "service_client");
    ros::NodeHandle nh;
    client = nh.serviceClient<std_srvs::Trigger>("service1", true);

    ros::SubscribeOptions opts;
    opts.init<std_msgs::Empty>("topic1", 100, subscriber1_callback);
    opts.transport_hints = ros::TransportHints().reliable(); // Set reliable transport
    // true will enable the specified mutex to be used, flase will use the internal lock
    opts.allow_concurrent_callbacks = true; 
    ros::Subscriber sub = nh.subscribe(opts);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}