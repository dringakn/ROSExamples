/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Description:
 *    Notes:
 *    Add find_package(OpenCV)
 *    Add cv_bridge and image_transport library.
 *    Add include_directories (include ${OpenCV_INCLUDE_DIRS})
 *    Add target_link_libraries (... ${OpenCV_LIBRARIES})
 **/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "example_opencv_capture_image");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/image", 1);
  cv::VideoCapture cap(0);
  if (!cap.isOpened()) return -1;

  ros::Rate rate(10);
  while (ros::ok()) {
    cv::Mat frame;
    cap >> frame;
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  if (cap.isOpened()) {
    cap.release();
  }

  return 0;
}