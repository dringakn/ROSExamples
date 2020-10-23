/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Multiple messages synchronization.
    Note: Only works with messages containing header (time information).

    The TimeSynchronizer filter synchronizes incoming channels by the timestamps
    contained in their headers, and outputs them in the form of a single
    callback that takes the same number of channels. The C++ implementation can
    synchronize up to 9 channels.
*/
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/connection.h>
#include <message_filters/pass_through.h>
#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>

using namespace geometry_msgs;
using namespace std;

void Callback(const PointStamped::ConstPtr& msgP,
              const Vector3Stamped::ConstPtr& msgV) {
  //   ROS_INFO_STREAM(*msgP);
  //   ROS_INFO_STREAM(*msgV);
  ROS_INFO("%f", ros::Time::now().toSec());
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_message_filter");
  ros::NodeHandle nh;
  message_filters::Subscriber<PointStamped> subPt(nh, "/point", 1);
  message_filters::Subscriber<Vector3Stamped> subVt(nh, "/vector", 1);
  message_filters::TimeSynchronizer<PointStamped, Vector3Stamped> sync(
      subPt, subVt, 10);
  sync.registerCallback(boost::bind(&Callback, _1, _2));
  ros::spin();
  return 0;
}