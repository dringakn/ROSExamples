/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Subscribes to a templated message.
                 One class to subscribe multiple type of message.
                 Calculate the timeperiod of messages.
                 Implement a message timeout timer.
                 Method to get available nodes, topics and master URI.
*/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;

namespace MySensor {
template <typename T>
class Sensor {
 private:
  ros::NodeHandle nh;              // Ros node handle
  ros::Time prevTime1, prevTime2;  // Time period calculation
  ros::Subscriber sub;             // Subscriber object
  ros::Timer timer;                // Message timer
  string strTopic;                 // Topic name
  int iExtraInfo;                  // Extra information
  double realTimePeriod;           // Real Timer period
  double expectedTimePeriod;       // Expected Timer period
  bool timeOut;                    // Specify if message timeout occures

 protected:
  /**
   * @brief Message callback function.
   *
   * @param msg
   */
  void msgCallback(const typename T::ConstPtr& msg) {
    timeOut = false;
    prevTime1 = prevTime2;
    prevTime2 = ros::Time::now();
    realTimePeriod = (prevTime2 - prevTime1).toSec();
    timer.setPeriod(ros::Duration(expectedTimePeriod), true);
    cout << strTopic << ": " << realTimePeriod << " sec" << endl;
    cout << *msg << endl;
  }

  /**
   * @brief The timer callback event
   *
   * @param event
   */
  void timerCallback(const ros::TimerEvent& event) {
    timeOut = true;
    cout << strTopic << ": "
         << "Timeout: " << timeOut << endl;
  }

 public:
  /**
   * @brief Construct a new Sensor object using node handle and topic name.
   *
   * @param n: ROS node handle
   * @param topic: topic to subscribe
   * @param timePeriod: message expected timer period
   */
  Sensor(ros::NodeHandle& n, string topic, double timePeriod = 1) {
    nh = n;
    strTopic = topic;
    expectedTimePeriod = timePeriod;
    iExtraInfo = 0;
    realTimePeriod = 0;
    timeOut = false;
    prevTime1 = prevTime2 = ros::Time::now();
    sub = nh.subscribe<T>(strTopic, 1, &Sensor::msgCallback, this);
    timer = nh.createTimer(ros::Duration(expectedTimePeriod),
                           &Sensor::timerCallback, this, false, true);
  }

  /**
   * @brief Get the real time Period between messages
   *
   * @return double
   */
  double getRealTimePeriod() const { return realTimePeriod; }

  /**
   * @brief Get the expected time Period between messages
   *
   * @return double
   */
  double getExpectedTimePeriod() const { return expectedTimePeriod; }

  /**
   * @brief Get the Subscirbed topic name
   *
   * @return string
   */
  string getTopic() const { return strTopic; }

  /**
   * @brief Get the list of nodes currently available.
   *
   * @return vector<string>
   */
  vector<string> getNodes() {
    vector<string> nodes;
    ros::master::getNodes(nodes);
    vector<string> result;
    for (auto&& n : nodes) result.push_back(n);
    return result;
  }

  /**
   * @brief Get the list of topics currently available.
   *
   * @return vector<string>
   */
  vector<string> getTopics() {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    vector<string> result;
    for (auto&& t : topics) result.push_back(t.name);
    return result;
  }

  /**
   * @brief Get the Host Name
   *
   * @return string
   */
  string getHost() { return ros::master::getHost(); }

  /**
   * @brief Get the Port Number
   *
   * @return string
   */
  string getPort() { return std::to_string(ros::master::getPort()); }

  /**
   * @brief Get the Host URI
   *
   * @return string
   */
  string getURI() { return ros::master::getURI(); }
};
}  // namespace MySensor

using namespace MySensor;

int main(int argc, char* argv[]) {
  cout.precision(5);
  ros::init(argc, argv, "example_message_subscriber_template");
  ros::NodeHandle nh;
  Sensor<std_msgs::String> s1(nh, "/test1", 1);
  Sensor<std_msgs::Int32> s2(nh, "/test2", 1);
  Sensor<std_msgs::Float32> s3(nh, "/test3", 1);
  vector<string> topics = s1.getTopics();
  vector<string> nodes = s1.getNodes();
  string host = s1.getHost();
  string port = s1.getPort();
  string uri = s1.getURI();
  cout << "URI: " << uri << endl;
  cout << "Host: " << host << endl;
  cout << "Port: " << port << endl;
  cout << "Nodes: " << endl;
  for (auto n : nodes) cout << n << endl;
  cout << "Topics: " << endl;
  for (auto t : topics) cout << t << endl;
  ros::spin();
  return 0;
}