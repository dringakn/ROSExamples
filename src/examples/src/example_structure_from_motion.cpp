/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
**/
#include <cv_bridge/cv_bridge.h>             // ROS and OpenCV interface
#include <geometry_msgs/Point32.h>           // PointCloud point
#include <image_transport/image_transport.h> // ROS and OpenCV image format conversion
#include <opencv-3.3.1-dev/opencv2/features2d/features2d.hpp> // SIFT/SURF Features
#include <opencv-3.3.1-dev/opencv2/opencv.hpp> // OpenCV libraries
#include <ros/ros.h>                           // ROS libraries
#include <sensor_msgs/Image.h>                 // Image message
#include <sensor_msgs/PointCloud.h>            // Estimated 3D Points

image_transport::Publisher imgPub;              // Resultant image publiser
ros::Publisher pcPub;                           // Estimed 3D points publisher
cv::Mat camIntrinsic = cv::Mat(3, 3, CV_32FC1); // Camera Intrinsics
cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1);   // Lense distortion coeffiecnts

void callback_Image(const sensor_msgs::ImageConstPtr &img) {
  cv_bridge::CvImagePtr cv_ptr; // ImagePtr for ROS & CV conversion
  cv::Mat grayImage, nextImage;

  // Parameters for Lucas-Kanade Optical flows
  static cv::Mat prevImage;
  static std::vector<cv::Point2f> nextPoints, prevPoints;
  static std::vector<uchar> status;
  static std::vector<float> error;
  static bool initFeatures = true;
  static cv::TermCriteria termCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20,
                                       0.03); // Type, MaxItrCount, Epislon
  static cv::Size subPixelWinSize(5, 5), windowSize(20, 20);
  const unsigned int MAX_FEATURES = 100;

  try {
    cv_ptr = cv_bridge::toCvCopy(
        img, sensor_msgs::image_encodings::BGR8); // Convert to opencv image
    // ROS_INFO("Image Size: %dX%d", img->width, img->height);
    cv::cvtColor(cv_ptr->image, grayImage,
                 cv::COLOR_BGR2GRAY); // Convert to gray scale image
    cv::undistort(grayImage, nextImage, camIntrinsic,
                  distCoeffs); // Undistort the input image
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (initFeatures) {
    // cv::goodFeaturesToTrack(nextImage, nextPoints, MAX_FEATURES, 0.01, 5,
                            // cv::Mat(), 3, 0, 0.04);
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
          cv::circle(cv_ptr->image, nextPoints[k], 5, CV_RGB(255, 0, 0), 1, 1,
                     0);
        } else {
          cv::line(cv_ptr->image, prevPoints[k], nextPoints[k],
                   CV_RGB(0, 0, 255), 3, 1, 0);
          cv::circle(cv_ptr->image, prevPoints[k], 5, CV_RGB(255, 0, 0), 1, 1,
                     0);
        }
        validFeatures++;
      }
    }
    if (validFeatures <= 0.1 * MAX_FEATURES)
      initFeatures = true;
    // ROS_INFO("x:%05.2f y:%05.2f status:%d error:%6.4f", nextPoints[k].x,
    // nextPoints[k].y, status[k], error[k]);
    ROS_INFO("Features Tracked: %d", validFeatures);

    // Structure from motion
    /* Try to find essential matrix from the points */
    cv::Mat fundamental =
        findFundamentalMat(nextPoints, prevPoints, cv::FM_RANSAC, 3.0, 0.99);
    fundamental.convertTo(fundamental, CV_32F);
    cv::Mat essential = camIntrinsic.t() * fundamental * camIntrinsic;
    /* Find the projection matrix between those two images */
    cv::SVD svd(essential);
    static const cv::Mat W =
        (cv::Mat_<float>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
    static const cv::Mat W_inv = W.inv();
    cv::Mat_<float> R1 = svd.u * W * svd.vt;
    cv::Mat_<float> T1 = svd.u.col(2);
    cv::Mat_<float> R2 = svd.u * W_inv * svd.vt;
    cv::Mat_<float> T2 = -svd.u.col(2);
    static const cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1); // First camera pose
    cv::Mat P2 =
        (cv::Mat_<float>(3, 4) << R1(0, 0), R1(0, 1), R1(0, 2), T1(0), R1(1, 0),
         R1(1, 1), R1(1, 2), T1(1), R1(2, 0), R1(2, 1), R1(2, 2), T1(2));
    std::cout << P2 << std::endl;
    /*  Triangulate the points to find the 3D homogenous points in the world
       space
        Note that each column of the 'out' matrix corresponds to the 3d
       homogenous point */
    cv::Mat out;
    cv::triangulatePoints(P1, P2, nextPoints, prevPoints, out);
    /* Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1)
     */
    // out.row(0) /= out.row(3);
    // out.row(1) /= out.row(3);
    // out.row(2) /= out.row(3);
    // std::cout << out << std::endl;
    static sensor_msgs::PointCloud pc;
    static int count = 0;
    const unsigned int nPoints = 100000;
    pc.header.frame_id = "/map";
    pc.header.stamp = ros::Time::now();
    pc.points.resize(nPoints);
    for (unsigned int i = 0; i < out.cols; i++) {
      double w = out.at<double>(3, i);
      if (w == std::numeric_limits<double>::infinity() || fabs(w) <= 1e-5)
        continue;
      geometry_msgs::Point32 pt;
      pt.x = 10 * out.at<double>(0, i) / w;
      pt.y = 10 * out.at<double>(1, i) / w;
      pt.z = 10 * out.at<double>(2, i) / w;
      // pc.points.push_back(pt);
      pc.points[count] = pt;
      if (count++ > nPoints)
        count = 0;
    }
    // cv::imshow("Image Window", cv_ptr->image);
    // cv::waitKey(100);
    imgPub.publish(cv_ptr->toImageMsg());
    pcPub.publish(pc);
  }
  nextImage.copyTo(prevImage);
  std::swap(nextPoints, prevPoints);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_image_subscriber");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe("/floorCamera", 1, callback_Image);
  imgPub = it.advertise("/output_image", 1);
  pcPub = nh.advertise<sensor_msgs::PointCloud>("pointCloud", 1, true);
  camIntrinsic = (cv::Mat_<float>(3, 3) << 1, 0, 256, 0, 1, 256, 0, 0,
                  1); // fx 0 cx;0 fy cy;0 0 1
  distCoeffs = (cv::Mat_<float>(1, 5) << 0e-5, 0, 0, 0, 0); // k1, k2, p1, p2,
                                                            // k3
  std::cout << camIntrinsic << std::endl;
  std::cout << distCoeffs << std::endl;
  ros::spin(); // Start processing
  return 0;
}
