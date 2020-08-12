/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        The following program create a 3D map using inertial and
        3D point cloud date.
    Notes:
        include_directory (... include ${PCL_INCLUDE_DIRS} ...)
        find_package(PCL 1.7 REQUIRED)
        find_package(catkin REQUIRED COMPONENTS
        roscpp sensor_msgs geometry_msgs tf pcl_ros nav_msgs random_numbers)
**/

#include <pcl/filters/voxel_grid.h>             // voxel filter
#include <pcl/segmentation/sac_segmentation.h>  // plane segmentation
#include <pcl_conversions/pcl_conversions.h>    // pcl::PCLPointCloud2, to/from
#include <pcl_msgs/ModelCoefficients.h>         // Model coefficents
#include <ros/ros.h>                            // ros

ros::Publisher pubPC, pubModel;

void pointCloud_cb(const pcl::PCLPointCloud2::ConstPtr &msg) {
  // output message
  pcl::PCLPointCloud2 out;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxFilter;
  voxFilter.setInputCloud(msg);
  voxFilter.setLeafSize(0.02, 0.02, 0.02);  // 2cm^3 voxel
  voxFilter.filter(out);
  // publish the data
  pubPC.publish(out);

  // Segement and extract plane parameters
  pcl::ModelCoefficients coeff;
  // Inliers point indices
  pcl::PointIndices inliers;
  // Segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);  // plane model
  seg.setMethodType(pcl::SAC_RANSAC);     // ransac method
  seg.setDistanceThreshold(0.01);
  // pcl::PCLPointCloud2ConstPtr ptr(&pc);
  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromPCLPointCloud2(out, pc);
  // pcl::toPCLPointCloud2(pc, out);
  seg.setInputCloud(pc.makeShared());
  seg.segment(inliers, coeff);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients msgCoeff;
  pcl_conversions::fromPCL(coeff, msgCoeff);
  // msgCoeff.header.frame_id = "/device";
  // msgCoeff.header.stamp = ros::Time(0);
  pubModel.publish(msgCoeff);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_ros_callback");
  ros::NodeHandle nh;
  // subscribe to the input point cloud
  ros::Subscriber subPC = nh.subscribe("tango/point_cloud", 1, pointCloud_cb);
  // create a voxel filtered output publisher
  pubPC = nh.advertise<pcl::PCLPointCloud2>("/pointcloud", 1, true);
  // publishe extracted model (plane) coefficents
  pubModel = nh.advertise<pcl_msgs::ModelCoefficients>("/modelcoeff", 1, true);

  ros::spin();
  return 0;
}