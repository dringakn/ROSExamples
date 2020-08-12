// Avoid including it multiple times, if needed, otherwise not including it.
#ifndef MY_NODE_CLASS_H
#define MY_NODE_CLASS_H

#include <ros/ros.h>          // ros funcationality
#include <std_msgs/String.h>  // string message
#include <std_srvs/Trigger.h> // service message
#include <string>             // std::to_string(val), c++11

// Class Defination
class MyNodeClass {
private:
  // Node handle required to be passed between main and constructor.
  ros::NodeHandle nh;
  // These shall be set up inside constructor to hide ugle details.
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceServer ser;
  // Some variables, better than using globals
  double valFromSer;
  std::string valFromSub;
  // Helper functions
  void initPublisher();
  void initSubscriber();
  void initService();
  // Prototype for subscriber callback
  void subCallback(const std_msgs::String::ConstPtr &msg);
  // Prototype for service callback, note the return type is bool
  bool serCallback(std_srvs::TriggerRequest &req,
                   std_srvs::TriggerResponse &res);

public:
  // Main shall initiate a nodehandle and pass it to the class
  MyNodeClass(ros::NodeHandle *nhPtr);
  virtual ~MyNodeClass();
}; // End of class defination

#endif