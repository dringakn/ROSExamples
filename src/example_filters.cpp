/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *    Note: Either use mean filter or increment filter as they have the same
 *          ifndef tag. Otherwise, modify the increment.h file to correct the
 *          ifndef tag. For the single channel mean filter, num_of_observations
 *          should be set on the rosparam server.
 **/

// #include <filters/increment.h>          // Pre-implemented ROS filter
#include <filters/mean.h>               // Pre-implemented ROS filter
#include <filters/median.h>             // Pre-implemented ROS filter
#include <filters/transfer_function.h>  // Pre-implemented ROS filter
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

int main(int argc, char* argv[]) {
  cout.precision(3);
  ros::init(argc, argv, "example_filters");
  ros::NodeHandle nh("~");
  random_numbers::RandomNumberGenerator rng;

  filters::RealtimeCircularBuffer<double> circBuff(10, 0);
  filters::FilterBase<double>* avgFilter = new filters::MeanFilter<double>();
  filters::FilterBase<double>* medFilter = new filters::MedianFilter<double>();
  avgFilter->configure("MeanFilter", nh);
  medFilter->configure("MedianFilter", nh);

  ros::Publisher pubMean = nh.advertise<std_msgs::Float64>("/mean", 1);
  ros::Publisher pubMedian = nh.advertise<std_msgs::Float64>("/median", 1);
  std_msgs::Float64 msgMean, msgMedian;

  double in;
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    in = rng.gaussian(0, 1);

    circBuff.push_back(in);
    cout << "Circular Buffer: ";
    for (int i = 0; i < circBuff.size(); i++)
      cout << circBuff[i] << ((i < circBuff.size() - 1) ? ", " : "\r\n");

    avgFilter->update(in, msgMean.data);
    cout << "Mean: " << in << " -> " << msgMean.data << endl;

    medFilter->update(in, msgMedian.data);
    cout << "Median: " << in << " -> " << msgMedian.data << endl;

    cout << endl;

    pubMean.publish(msgMean);
    pubMedian.publish(msgMedian);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}