/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Notes:
      rosrun nodelet declared_nodelets
      rospack plugins --attrib=plugin nodelet
      roslaunch ros_examples example_nodelet.launch
      If the nodelet is not added as a dependency inside package.xml file, it
      shall not be visible by the nodelet.
*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace nodelet_example {
class Hello : public nodelet::Nodelet {
 private:
  ros::Publisher pub_;
  ros::Subscriber sub_;
  virtual void onInit() {
    ros::NodeHandle& pnh = getPrivateNodeHandle();
    NODELET_INFO("Nodelet initialize");
    pub_ = pnh.advertise<std_msgs::String>("msg_out", 1, false);
    sub_ = pnh.subscribe<std_msgs::String>("msg_in", 1, &Hello::callback, this);
  }
  void callback(const std_msgs::String::ConstPtr& msg) {
    std_msgs::String m;
    m.data = msg->data;
    pub_.publish(m);
  }
};
};  // namespace nodelet_example

PLUGINLIB_EXPORT_CLASS(nodelet_example::Hello, nodelet::Nodelet)
