/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Multiple messages synchronization.
      The TimeSynchronizer filter synchronizes incoming channels by the timestamps
      contained in their headers, and outputs them in the form of a single
      callback that takes the same number of channels. The C++ implementation can
      synchronize up to 9 channels (topics).

      The example wait for two message topics (/point, /vector) and synchronize
      them interm of time. The output message is published at the rate of smallest
      frequency of the two messages. For example if one of the message is being 
      published @ 1Hz while the other is at 0.2Hz then the output callback is called
      at the rate of 0.5Hz.

      Example:
        roscore
        rosrun ros_examples example_message_filter
        rostopic pub -r 1 /point geometry_msgs/PointStampled ... 
        rostopic pub -r 0.2 /vector geometry_msgs/Vector3Stampled ... 

      Inputs are connected either through the filter's constructor or through 
      the connectInput() method. Outputs are connected through the registerCallback() method. 

      You can register multiple callbacks with the registerCallbacks() method. 
      They will get called in the order they are registered.

      The TimeSequencer filter guarantees that messages will be called in temporal order according 
      to their header's timestamp. The TimeSequencer is constructed with a specific delay which specifies 
      how long to queue up messages before passing them through. A callback for a message is never invoked 
      until the messages' time stamp is out of date by at least delay. However, for all messages which are 
      out of date by at least the delay, their callback are invoked and guaranteed to be in temporal order. 
      If a message arrives from a time prior to a message which has already had its callback invoked, 
      it is thrown away. 

      Cache, stores a time history of messages.
      Given a stream of messages, the most recent N messages are cached in a ring buffer, 
      from which time intervals of the cache can then be retrieved by the client. 
      The timestamp of a message is determined from its header field.
      The Cache immediately passes messages through to its output connections. 
      The user can then make calls like cache.getInterval(start, end) to extract part of the cache. 

      The Chain filter allows you to dynamically chain together multiple single-input/single-output (simple) filters. 
      As filters are added to it they are automatically connected together in the order they were added. 
      It also allows you to retrieve added filters by index.
      Chain is most useful for cases where you want to determine which filters to apply at runtime rather than compile-time. 
    Note: 
      Only works with messages containing header (time information).
      The TimeSynchronizer filter synchronizes incoming channels by the timestamps contained in their headers, 
      and outputs them in the form of a single callback that takes the same number of channels. 
*/

#include <ros/ros.h> // ROS
#include <message_filters/connection.h>
#include <message_filters/pass_through.h>
#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PointStamped.h>   // Point message
#include <geometry_msgs/Vector3Stamped.h> // Vector message

using namespace geometry_msgs;
using namespace std;

void Callback(const PointStamped::ConstPtr &msgP,
              const Vector3Stamped::ConstPtr &msgV)
{
  //   ROS_INFO_STREAM(*msgP);
  //   ROS_INFO_STREAM(*msgV);
  ROS_INFO("%f", ros::Time::now().toSec());
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "example_message_filter");
  ros::NodeHandle nh;
  message_filters::Subscriber<PointStamped> subPt(nh, "/point", 1);
  message_filters::Subscriber<Vector3Stamped> subVt(nh, "/vector", 1);
  using syncPolicyA = message_filters::sync_policies::ApproximateTime<PointStamped, Vector3Stamped>;
  using syncPolicyE = message_filters::sync_policies::ExactTime<PointStamped, Vector3Stamped>;
  // message_filters::TimeSynchronizer<PointStamped, Vector3Stamped> sync(subPt, subVt, 10); // 10: queue size
  message_filters::Synchronizer<syncPolicyA> sync(syncPolicyA(10), subPt, subVt);
  sync.registerCallback(boost::bind(&Callback, _1, _2));
  // sync.registerCallback(Callback);
  ros::spin();
  return 0;
}