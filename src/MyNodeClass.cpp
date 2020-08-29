/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "MyNodeClass.h"

MyNodeClass::MyNodeClass(ros::NodeHandle *nhPtr)
    : nh(*nhPtr),      // Initialize node handle member variable
      valFromSub(""),  // Initialize the private variable as required
      valFromSer(0)    // Initialize the private variable as required
{
  ROS_INFO("MyNodeClass constructor initialized.");
  initSubscriber();
  initPublisher();
  initService();
}

MyNodeClass::~MyNodeClass() {}

void MyNodeClass::initPublisher() {
  ROS_INFO("MyNodeClass publishers initialized.");
  pub = nh.advertise<std_msgs::String>("topic_pub", 1, true);
}

void MyNodeClass::initSubscriber() {
  ROS_INFO("MyNodeClass subscribers initialized.");
  // sub is a pointer to a member function "subCallback" of MyNodeClass
  // therefore, "this" keyword is required to access the current instance of
  // MyNodeClass
  sub = nh.subscribe("topic_sub", 1, &MyNodeClass::subCallback, this);
}

void MyNodeClass::initService() {
  ROS_INFO("MyNodeClass services initialized.");
  ser = nh.advertiseService("service_name", &MyNodeClass::serCallback, this);
}

void MyNodeClass::subCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("MyNodeClass subscriber callback.");
  valFromSub = msg->data;
  std_msgs::String outmsg;
  outmsg.data = valFromSub + "_sub";
  pub.publish(outmsg);
}

bool MyNodeClass::serCallback(std_srvs::TriggerRequest &req,
                              std_srvs::TriggerResponse &res) {
  ROS_INFO("MyNodeClass service callback.");
  // req has no member fields, res has two member fields which are as follows
  valFromSer++;
  res.success = true;
  res.message = "response string:" + std::to_string(valFromSer);
  return true;
}