/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *          The following example shows how to create mean/median/digital
 *          filters (single/multi channels). Furthermore, how to concatenate
 *          different filters (e.g. increment filter) using chain.
 *          Filter chain can be used to implement band-pass filter.
 *          chain works with built-in filters such as
 *            filters/IncrementFilterInt
 *            filters/MeanFilterFloat
 *            filters/MultiChannelMedianFilterDouble
 *          or with dervied filters as plugin.
 *
 *    Note: Either use mean filter or increment filter as they have the same
 *          ifndef tag. Otherwise, modify the increment.h file to correct the
 *          ifndef tag. Use plotjuggler to visulize the data.
 *          Filter configurations are stored in filters.yaml file.
 *
 *          roslaunch ros_examples example_filters.launch
 *
 **/

#include <filters/filter_chain.h>           // Filter chain
#include <filters/increment.h>              // Pre-implemented ROS filter
#include <filters/mean.h>                   // Pre-implemented ROS filter
#include <filters/median.h>                 // Pre-implemented ROS filter
#include <filters/transfer_function.h>      // Pre-implemented ROS filter
#include <random_numbers/random_numbers.h>  // Random number generator
#include <ros/ros.h>                        // ROS functionality
#include <std_msgs/Float32MultiArray.h>     // Array message
#include <std_msgs/Float64.h>               // double message

using namespace std;
using namespace filters;

int main(int argc, char* argv[]) {
  cout.precision(3);
  ros::init(argc, argv, "example_filters");
  ros::NodeHandle nh("~");
  random_numbers::RandomNumberGenerator rng;
  double update_rate = 1;
  nh.getParam("update_rate", update_rate);

  const int CHANNELS = 3;  // Number of channels for multi-channel filter

  // Circular buffer
  RealtimeCircularBuffer<double> circBuff(10, 0);

  // Single channel mean/average filter
  FilterBase<double>* avgFilter = new MeanFilter<double>();
  avgFilter->configure("MeanFilter", nh);  // Private nh, wrt to node ns

  // Multi-channel mean/average filter
  MultiChannelFilterBase<double>* avgMFilter =
      new MultiChannelMeanFilter<double>();
  avgMFilter->configure(CHANNELS, "MultiChannelMeanFilter", nh);

  //  Single channel median filter
  FilterBase<double>* medFilter = new MedianFilter<double>();
  medFilter->configure("MedianFilter", nh);

  // Multi-channel median filter
  MultiChannelFilterBase<double>* medMFilter =
      new MultiChannelMedianFilter<double>();
  medMFilter->configure(CHANNELS, "MultiChannelMedianFilter", nh);

  /*
  Single-channel low-pass transfer function (digital) filter:

    a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1]+ ... + b[n_b]*x[n-n_b]
                          - a[1]*y[n-1]- ... - a[n_a]*y[n-n_a]
    If a[0] is not equal to 1, the coefficients are normalized by a[0].

    Example config:
    <filter type="TransferFunctionFilter" name="filter_name">
        <params a="1.0 0.5" b="0.2 0.2"/>
    </filter>
  */
  FilterBase<double>* tfFilter =
      new SingleChannelTransferFunctionFilter<double>();
  tfFilter->configure("SingleChannelLowPass", nh);

  // Multi-channel low-pass transfer function filter
  MultiChannelFilterBase<double>* tfMFilter =
      new MultiChannelTransferFunctionFilter<double>();
  tfMFilter->configure(CHANNELS, "MultiChannelLowPass", nh);

  // Increment filter for filter chain testing
  FilterBase<double>* incFilter = new IncrementFilter<double>();
  incFilter->configure("IncrementFilter", nh);

  // Multi-chain Increment filter for filter chain testing
  MultiChannelFilterBase<int>* incMFilter =
      new MultiChannelIncrementFilter<int>();
  incMFilter->configure(CHANNELS, "MultiChannelIncrementFilter", nh);

  // Chain filter
  FilterChain<int> chainFilter("int");
  chainFilter.configure("ThreeIncrements", nh);

  // Topics and messages for publishing
  ros::Publisher pubInc = nh.advertise<std_msgs::Float64>("/inc_filter", 1);
  ros::Publisher pubMean = nh.advertise<std_msgs::Float64>("/mean_filter", 1);
  ros::Publisher pubMedian =
      nh.advertise<std_msgs::Float64>("/median_filter", 1);
  ros::Publisher pubTF = nh.advertise<std_msgs::Float64>("/digital_filter", 1);
  ros::Publisher pubMInc =
      nh.advertise<std_msgs::Float32MultiArray>("/multi_inc_filter", 1);
  ros::Publisher pubMMean =
      nh.advertise<std_msgs::Float32MultiArray>("/multi_mean_filter", 1);
  ros::Publisher pubMMedian =
      nh.advertise<std_msgs::Float32MultiArray>("/multi_median_filter", 1);
  ros::Publisher pubMTF =
      nh.advertise<std_msgs::Float32MultiArray>("/multi_digital_filter", 1);

  std_msgs::Float64 msgMean, msgMedian, msgTF, msgInc;
  std_msgs::Float32MultiArray msgMMean, msgMMedian, msgMTF, msgMInc;
  msgMMean.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msgMMedian.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msgMTF.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msgMInc.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msgMMean.layout.data_offset = msgMMedian.layout.data_offset =
      msgMTF.layout.data_offset = msgMInc.layout.data_offset = 0;  // offset
  msgMMean.layout.dim[0].label = "MeanFiltered";                   // Array name
  msgMMedian.layout.dim[0].label = "MedianFiltered";
  msgMTF.layout.dim[0].label = "DigitalFiltered";
  msgMInc.layout.dim[0].label = "IncFiltered";
  msgMMean.layout.dim[0].size = msgMMedian.layout.dim[0].size =
      msgMTF.layout.dim[0].size = msgMInc.layout.dim[0].size =
          CHANNELS;  // Number of elements
  msgMMean.layout.dim[0].stride = msgMMedian.layout.dim[0].stride =
      msgMTF.layout.dim[0].stride = msgMInc.layout.dim[0].stride =
          CHANNELS * 4;              // Number of bytes
  msgMInc.data.resize(CHANNELS);     // Adjust array size
  msgMMean.data.resize(CHANNELS);    // Adjust array size
  msgMMedian.data.resize(CHANNELS);  // Adjust array size
  msgMTF.data.resize(CHANNELS);      // Adjust array size

  double in, out;                    // single-channel filter input/output
  vector<double> inputs(CHANNELS);   // multi-channel filter inputs
  vector<double> outputs(CHANNELS);  // multi-channel filter outputs

  ros::Rate loop_rate(update_rate);

  while (ros::ok()) {
    // Generate a single-channel filter sample
    in = rng.gaussian(0, 1);

    // Generate multi-channel filter sample
    for (int i = 0; i < CHANNELS; i++) inputs[i] = rng.gaussian(0, 1);

    // Add it to the circular buffer
    circBuff.push_back(in);
    cout << "Circular Buffer: ";
    for (int i = 0; i < circBuff.size(); i++)
      cout << circBuff[i] << ((i < circBuff.size() - 1) ? ", " : "\r\n");

    // Pass it to the mean/average filter ang get the filtered output
    avgFilter->update(in, msgMean.data);
    avgMFilter->update(inputs, outputs);
    cout << "Mean  : " << in << " -> " << msgMean.data << endl;
    for (int i = 0; i < CHANNELS; i++) {
      cout << "Mean-" << i << ": " << inputs[i] << " -> " << outputs[i] << endl;
      msgMMean.data[i] = outputs[i];
    }

    // Pass it to the median filter ang get the filtered output
    medFilter->update(in, msgMedian.data);
    medMFilter->update(inputs, outputs);
    cout << "Median  : " << in << " -> " << msgMedian.data << endl;
    for (int i = 0; i < CHANNELS; i++) {
      cout << "Median-" << i << ": " << inputs[i] << " -> " << outputs[i]
           << endl;
      msgMMedian.data[i] = outputs[i];
    }

    // Pass it to the low-pass tf filter ang get the filtered output
    tfFilter->update(in, msgTF.data);
    tfMFilter->update(inputs, outputs);
    cout << "TF  : " << in << " -> " << msgTF.data << endl;
    for (int i = 0; i < CHANNELS; i++) {
      cout << "TF-" << i << ": " << inputs[i] << " -> " << outputs[i] << endl;
      msgMTF.data[i] = outputs[i];
    }

    // Pass it to the increment filter ang get the incremented output (in+1)
    incFilter->update(in, msgInc.data);
    std::vector<int> vin(begin(inputs), end(inputs)), vout(CHANNELS);
    incMFilter->update(vin, vout);
    cout << "INC  : " << in << " -> " << msgInc.data << endl;
    for (int i = 0; i < CHANNELS; i++) {
      cout << "INC-" << i << ": " << vin[i] << " -> " << vout[i] << endl;
      msgMInc.data[i] = vout[i];
    }

    // Pass it to the chain filter ang get the incremented output
    int intIn = 1, intOut = 0;
    chainFilter.update(intIn, intOut);
    cout << "CHAIN  : " << intIn << " -> " << intOut << endl;
    // chainFilter.update(in, out);
    // cout << "CHAIN  : " << in << " -> " << out << endl;

    cout << endl;

    // Publish it on respective topics
    pubMean.publish(msgMean);
    pubMedian.publish(msgMedian);
    pubTF.publish(msgTF);
    pubMMean.publish(msgMMean);
    pubMMedian.publish(msgMMedian);
    pubMTF.publish(msgMTF);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}