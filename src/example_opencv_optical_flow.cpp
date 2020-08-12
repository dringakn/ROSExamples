/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
// Add find_package(OpenCV)
// Add cv_bridge and image_transport library.
// Add include_directories (include ${OpenCV_INCLUDE_DIRS})
// Add target_link_libraries (... ${OpenCV_LIBRARIES})

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>  // Image message

#include <opencv2/features2d/features2d.hpp>  // SIFT/SURF Features
#include <opencv2/opencv.hpp>

image_transport::Publisher pub;
cv::Mat camIntrinsic = cv::Mat(3, 3, CV_32FC1);  // Camera Intrinsics
cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1);    // Lense distortion coeffiecnts

std::vector<cv::KeyPoint> imgKeypoints;
cv::Mat imgDescriptors;
// cv::Ptr<cv::FeatureDetector> detector =
// cv::Ptr<cv::FeatureDetector>(new cv::SURF());
// cv::Ptr<cv::DescriptorExtractor> extractor =
// cv::Ptr<cv::DescriptorExtractor>(new cv::SURF());

void callback_Image(const sensor_msgs::ImageConstPtr &img) {
  cv_bridge::CvImagePtr cv_ptr;  // ImagePtr for ROS & CV conversion
  cv::Mat nextImage, grayImage;
  static cv::Mat prevImage;
  static std::vector<cv::Point2f> nextPoints, prevPoints;
  static std::vector<uchar> status;
  static std::vector<float> error;
  static bool initFeatures = true;
  static cv::TermCriteria termCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20,
                                       0.03);  // Type, MaxItrCount, Epislon
  static cv::Size subPixelWinSize(5, 5), windowSize(10, 10);
  unsigned int MAX_FEATURES = 100;
  try {
    cv_ptr = cv_bridge::toCvCopy(
        img, sensor_msgs::image_encodings::BGR8);  // Convert to opencv image
    // ROS_INFO("Image Size: %dX%d", img->width, img->height);
    cv::cvtColor(cv_ptr->image, grayImage,
                 cv::COLOR_BGR2GRAY);  // Convert to gray scale image
    cv::undistort(grayImage, nextImage, camIntrinsic,
                  distCoeffs);  // Undistort the input image
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (initFeatures) {
    // cv::goodFeaturesToTrack ( nextImage, nextPoints, MAX_FEATURES, 0.01, 5,
    // cv::Mat(), 3, 0, 0.04 );
    // detector->detect(grayImage, imgKeypoints);
    // extractor->compute(grayImage, imgKeypoints, imgDescriptors);
    nextPoints.clear();
    MAX_FEATURES = imgKeypoints.size();
    for (unsigned int k = 0; k < MAX_FEATURES; k++)
      nextPoints.push_back(imgKeypoints[k].pt);
    ROS_INFO("Points Detected: %d", (int)nextPoints.size());
    prevPoints = nextPoints;
    initFeatures = false;
  } else {
    cv::calcOpticalFlowPyrLK(prevImage, nextImage, prevPoints, nextPoints,
                             status, error, windowSize, 3, termCriteria, 0,
                             0.001);
    int validFeatures = 0;
    for (unsigned int k = 0; k < status.size(); k++) {
      if (status[k]) {
        if (nextPoints[k].x - prevPoints[k].x > 0) {
          cv::line(cv_ptr->image, nextPoints[k], prevPoints[k],
                   CV_RGB(0, 0, 255), 3, 1, 0);
          // cv::circle ( cv_ptr->image, nextPoints[k], 5, CV_RGB ( 255, 0, 0 ),
          // 1, 1, 0 );
          cv::circle(cv_ptr->image, nextPoints[k],
                     imgKeypoints[k].size * 1.2 * 2 / 9., CV_RGB(255, 0, 0), 1,
                     8, 0);
        } else {
          cv::line(cv_ptr->image, prevPoints[k], nextPoints[k],
                   CV_RGB(0, 0, 255), 3, 1, 0);
          // cv::circle ( cv_ptr->image, prevPoints[k], 5, CV_RGB ( 0, 0, 255 ),
          // 1, 1, 0 );
          cv::circle(cv_ptr->image, nextPoints[k],
                     imgKeypoints[k].size * 1.2 * 2 / 9., CV_RGB(255, 0, 0), 1,
                     8, 0);
        }
        validFeatures++;
      }
      // ROS_INFO("x:%05.2f y:%05.2f status:%d error:%6.4f", nextPoints[k].x,
      // nextPoints[k].y, status[k], error[k]);
      ROS_INFO("Features Tracked: %d", validFeatures);
    }
    if (validFeatures <= 0.1 * MAX_FEATURES) initFeatures = true;

    cv::imshow("Image Window", cv_ptr->image);
    cv::waitKey(1);
    // pub.publish(cv_ptr->toImageMsg());
  }
  nextImage.copyTo(prevImage);
  std::swap(nextPoints, prevPoints);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lab7_optical_flow");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe("/floorCamera", 1, callback_Image);
  pub = it.advertise("/output_image", 1);
  camIntrinsic = (cv::Mat_<float>(3, 3) << 1, 0, 256, 0, 1, 256, 0, 0,
                  1);  // fx 0 cx;0 fy cy;0 0 1
  distCoeffs = (cv::Mat_<float>(1, 5) << 0e-5, 0, 0, 0, 0);  // k1, k2, p1, p2,
                                                             // k3
  ros::spin();
  return 0;
}
