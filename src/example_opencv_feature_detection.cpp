/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
// Add find_package(OpenCV)
// Add cv_bridge and image_transport library.
// Add include_directories (include ${OpenCV_INCLUDE_DIRS})
// Add target_link_libraries (... ${OpenCV_LIBRARIES})

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/calib3d/calib3d.hpp>        // for homography
#include <opencv2/features2d/features2d.hpp>  // SURF Features
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
// #include <opencv2/nonfree/features2d.hpp> // SIFT Features

int main(int argc, char **argv) {
  ros::init(argc, argv, "webcam_image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/image", 1);
  ros::Rate rate(10);
  cv::VideoCapture cap(0);

  if (!cap.isOpened()) return -1;

  std::vector<cv::KeyPoint> objectKeypoints;
  cv::Mat objectDescriptors;
  // The detector can be any of (see OpenCV features2d.hpp):
  // cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
  // cv::FeatureDetector * detector = new cv::FastFeatureDetector();
  // cv::FeatureDetector * detector = new cv::GFTTDetector();
  // cv::FeatureDetector * detector = new cv::MSER();
  // cv::FeatureDetector * detector = new cv::ORB();
  // cv::Ptr<cv::FeatureDetector> detector = cv::Ptr<cv::FeatureDetector>(new
  // cv::ORB());
  //   cv::FeatureDetector *detector = new cv::SIFT();
  // cv::FeatureDetector * detector = new cv::StarFeatureDetector();
  // cv::FeatureDetector *detector = new cv::SURF(600.0);
  // cv::FeatureDetector * detector = new cv::BRISK();

  // The extractor can be any of (see OpenCV features2d.hpp):
  // cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
  // cv::DescriptorExtractor * extractor = new cv::ORB();
  // cv::Ptr<cv::DescriptorExtractor> extractor =
  // cv::Ptr<cv::DescriptorExtractor>(new cv::ORB());
  //   cv::DescriptorExtractor *extractor = new cv::SIFT();
  //   cv::DescriptorExtractor *extractor = new cv::SURF(600.0);
  // cv::DescriptorExtractor * extractor = new cv::BRISK();
  // cv::DescriptorExtractor * extractor = new cv::FREAK();

  while (ros::ok()) {
    cv::Mat frame;
    cap >> frame;
    // detector->detect(frame, objectKeypoints);
    // extractor->compute(frame, objectKeypoints, objectDescriptors);

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  if (cap.isOpened()) cap.release();

  return 0;
}
