/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Append /etc/security/limits.conf
        ahmad soft rtprio 99
        ahmad hard rtprio 99
        ahmad soft memlock unlimited
        ahmad hard memlock unlimited
    Reboot
    Verify using htop.
*/
#include <ros/ros.h>
#include <sys/mman.h>
#include <sched.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_node_real_time_priority");
    ros::NodeHandle nh;

    // Lock memory to prevent paging
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        ROS_ERROR("Failed to lock memory: %s", strerror(errno));
        return 1;
    }

    // Set scheduling policy and priority (0-99)
    struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        ROS_ERROR("Failed to set scheduling policy and priority: %s", strerror(errno));
        return 1;
    }

    return 0;
}