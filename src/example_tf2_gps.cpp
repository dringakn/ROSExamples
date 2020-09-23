/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      Publish a static transform from "world" to "map" frame
      use view_frames to view the tf graph (evince frames.pdf)
      or rosrun tf tf_monitor world map
      or rosrun tf tf_echo world map
      or rostopic echo /tf
      To publishe a static transform between two frames use the following
      rosrun tf static_transform_publisher x y z yaw pitch roll parent child t_ms
    Notes:
        The class tf2::Transform has built-in implementation of multiplication operator (*)
        for the transformation of Vector3, quaternion and transformation object.
        For example:
        tf2::Vector3 v(p.x, p.y, p.z);
        v = t2 * v;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        - The tf2_ros::TransformBroadcaster and tf2_ros::TransformLister are the tf2 equivlents
          of tf::TransformBroadcaster and tf::TransformListener respectively.
        - tf2_ros::TransformBroadcaster::sendTransform(msg) can be used directly.
        - tf2_ros::TransformListener is used to subscribe to /tf topic.
        - tf2_ros::Buffer::transform() is the main method for applying transforms.
        - canTransform() allows to know if a transform is available
        - lookupTransform is a lower level method which returns the transform between two coordinate frames.
          This method is the core functionality of the tf2 library.
        - getFrames is a service method providing the frames in the graph as a yaml tree.
        - tf2_ros provides a feature which allows to pass only the messages once there is transform data available.
          This follows the pattern from the message_filters package.
          Here is a brief list of functions that the user is most likely to use.
        - geometry_msgs::Transform and geometry_msgs::TransformStamped messages can be used for
          tf related messages.
        - The tfTransformListener functionality is implemented in tfBuffer in tf2.
        - The transform object for math operations are available in tf2/LinearMath/Transform
        - geometry_msgs::* including Transform messages can be directly printed using cout.
        - tf2::fromMsg(quat_tf, quat_msg);
**/

// #include <tf2/transform_datatypes.h>
#include <nav_msgs/Odometry.h>                    //Odometry message
#include <ros/ros.h>                              // ROS functionality
#include <tf2/LinearMath/Transform.h>             // TF2 functionality
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // TF2 <-> geometry_msgs conversion
#include <tf2_ros/transform_broadcaster.h>        // Transform broadcaster
#include <tf2_ros/transform_listener.h>           // Transform listener
using namespace std;

tf2_ros::TransformBroadcaster *br = NULL;  // Transform broadcaster object
tf2_ros::Buffer *tfBuffer = NULL;          // Transform listen buffer

// Custom formatted output instead of using cout
void printTF(geometry_msgs::TransformStamped &t)
{
  ROS_INFO("Pos: [%+05.2f, %+05.2f, %+05.2f], Rot: [%+05.2f, %+05.2f, %+05.2f, %+05.2f]", t.transform.translation.x,
           t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x, t.transform.rotation.y,
           t.transform.rotation.z, t.transform.rotation.w);
}

// Print the transform
void printTF(tf2::Transform &t)
{
  ROS_INFO("Pos: [%+05.2f, %+05.2f, %+05.2f], Rot: [%+05.2f, %+05.2f, %+05.2f, %+05.2f]", t.getOrigin().x(),
           t.getOrigin().y(), t.getOrigin().z(), t.getRotation().x(), t.getRotation().y(), t.getRotation().z(),
           t.getRotation().w());
}

/**
 * @brief Odometry callback function.
 *  Publish a tf world->map
 *
 * @param msg
 */
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  //   Listen for map to world transformation
  try
  {
    // t_wm = t_ws * t_sm
    // where t_ws is given bt the gps
    // find the latest (or at that time) t_sm
    //
    geometry_msgs::TransformStamped m;
    m = tfBuffer->lookupTransform("sensor", "map", ros::Time(0));
    tf2::Stamped<tf2::Transform> t_sm;
    tf2::fromMsg(m, t_sm);  // Convert message to tfstamped object
    tf2::Transform t_ws;
    tf2::fromMsg(msg->pose.pose, t_ws);  // Convert message to tf object
    tf2::Transform t_wm = t_ws * t_sm;
    printTF(t_wm);
    // Create transform message
    m.header.stamp = ros::Time::now();
    m.header.frame_id = msg->header.frame_id;
    m.child_frame_id = "map";
    m.transform.translation.x = t_wm.getOrigin().getX();
    m.transform.translation.y = t_wm.getOrigin().getY();
    m.transform.translation.z = t_wm.getOrigin().getZ();
    m.transform.rotation.x = t_wm.getRotation().getX();
    m.transform.rotation.y = t_wm.getRotation().getY();
    m.transform.rotation.z = t_wm.getRotation().getZ();
    m.transform.rotation.w = t_wm.getRotation().getW();
    br->sendTransform(m);  // Broadcast the transform to ROS
  }
  catch (tf2::TransformException ex)
  {
    ROS_INFO("%s", ex.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_tf2_gps");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/gps", 10, odom_cb);

  br = new tf2_ros::TransformBroadcaster();
  tfBuffer = new tf2_ros::Buffer(ros::Duration(10), false);
  tf2_ros::TransformListener li(*tfBuffer, nh, true);  // Transform listener object, why we need it in tf2?

  // Basic coordinate math
  // Compose two transforms; map->robot, robot->sensor
  tf2::Transform t_m_r(tf2::Quaternion(0, 0, 0), tf2::Vector3(10, 0, 0));
  tf2::Transform t_r_s(tf2::Quaternion(0, 0, 0), tf2::Vector3(1, 0, 0));
  tf2::Transform t_m_s = t_m_r * t_r_s;
  printTF(t_m_s);

  // Quaternion Basics
  // tf2::Quaternion q(0, 0, 0, 0);
  // ROS_INFO("Q: [%+05.2f, %+05.2f, %+05.2f, %+05.2f]", q.getX(), q.getY(), q.getZ(), q.getW());
  // ROS_INFO("Q_mag: [%+05.2f]", q.length());
  // q.normalize();
  // ROS_INFO("Q: [%+05.2f, %+05.2f, %+05.2f, %+05.2f]", q.getX(), q.getY(), q.getZ(), q.getW());

  // Get the tf between map -> robot
  tf2::Transform t_s_r = t_r_s.inverse();
  tf2::Transform t = t_m_s * t_s_r;  // Should be t_m_r
  printTF(t);

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}