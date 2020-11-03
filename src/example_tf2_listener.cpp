/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      TF2 transform listener example. The listener object requires a
      tfbuffer. The listener object doesn't have any members, however, the
      TF2 buffer contains the tf tree information.

    Notes:
      - Publish a static transform from "world" to "map" frame
      - use view_frames to view the tf graph (evince frames.pdf)
      - or rosrun tf tf_monitor world map
      - or rosrun tf tf_echo world map
      - or rostopic echo /tf
      - To publishe a static transform between two frames use the following
      - rosrun tf static_transfrm_publisher x y z yaw pitch roll part child t_ms

      The latest ROS versions uses tf2 under the hood of tf.
      tf::Transform can be used to do the math algebra operations.
      tf2 should be used as it has a cleaner interface.
      There are two helper classes to send and receive transforms published on
      ROS, tf::TransformBroadcaster and tf::TransformListener.
      tf Quternion, Vector3, Point, Transform, Pose are the typedef of bullet
      types. The stamped version has stamp, frame_id_, child_frame_id fields.
      ROS uses two quaternion datatypes: msg and 'tf.' To convert between them
      in C++, use the methods of tf2_geometry_msgs.
        #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

      The class tf2::Transform has built-in implementation of multiplication
      operator (*) for the transformation of Vector3, quaternion and
      transformation object.

      For example: tf2::Vector3 v(p.x, p.y, p.z);
      v = t2  * v; p.x = v.x(); p.y = v.y(); p.z = v.z();
    - The tf2_ros::TransformBroadcaster and tf2_ros::TransformLister are
      the tf2 equivlents of tf::TransformBroadcaster and tf::TransformListener
      respectively.
    - tf2_ros::TransformBroadcaster::sendTransform(msg) can be used
      directly.
    - tf2_ros::TransformListener is used to subscribe to /tf topic.
    - tf2_ros::Buffer::transform() is the main method for applying transforms.
    - canTransform() allows to know if a transform is available
    - lookupTransform is a lower level method which returns the transform
      between two coordinate frames. This method is the core functionality of
      the tf2 library.
    - getFrames is a service method providing the frames in the graph as a yaml
      tree.
    - tf2_ros provides a feature which allows to pass only the messages once
      there is transform data available. This follows the pattern from the
      message_filters package. Here is a brief list of functions that the user
      is most likely to use.
    - geometry_msgs::Transform and geometry_msgs::TransformStamped messages
      can be used for tf related messages.
    - The tfTransformListener functionality is implemented in tfBuffer in tf2.
    - The transform object for math operations are available in
      tf2/LinearMath/Transform
    - geometry_msgs::* including Transform messages can be directly printed
      using cout.
    - tf2::fromMsg(quat_tf, quat_msg);
**/

#include <ros/ros.h>                              // ROS functionality
#include <tf2/LinearMath/Transform.h>             // TF2 functionality
#include <tf2/transform_datatypes.h>              // TF2 geometry_msgs compatibility
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // TF2 <-> geometry_msgs conversion
#include <tf2_ros/transform_listener.h>           // Transform listener

using namespace std;

tf2_ros::Buffer *tfBuffer = NULL;  // Transform listen buffer

// Custom formatted output instead of using cout
void printTF(geometry_msgs::TransformStamped &t)
{
  ROS_INFO("Pos: [%+05.2f, %+05.2f, %+05.2f], Rot: [%+05.2f, %+05.2f, %+05.2f, "
           "%+05.2f]",
           t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, t.transform.rotation.x,
           t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
}

// Print the transform
void printTF(tf2::Transform &t)
{
  ROS_INFO("Pos: [%+05.2f, %+05.2f, %+05.2f], Rot: [%+05.2f, %+05.2f, %+05.2f, "
           "%+05.2f]",
           t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(), t.getRotation().x(), t.getRotation().y(),
           t.getRotation().z(), t.getRotation().w());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_tf2_listener");
  ros::NodeHandle nh("~");

  // Create a TF buffer object
  tfBuffer = new tf2_ros::Buffer(ros::Duration(10), false);
  // Create a TF listener object to fill buffer
  tf2_ros::TransformListener li(*tfBuffer, nh, true);
  // error message string for canTransform and lookup transform
  string errorTF;
  // parent frame query
  string parent;

  // Transform object for publishing
  geometry_msgs::TransformStamped t;
  t.child_frame_id = "sensor2";
  t.header.frame_id = "base_link";
  t.transform.rotation.w = 1;  // Normalized quaternion
  ros::V_string frames;

  ros::Rate rate(1);
  while (ros::ok())
  {
    /*
      Send a transform without using the TF broadcaster object.
      The third object is used to specifiy if the transform is static or not.
      In case of a static tf, the rate is set to 10000.
      Another way of specifying the tf as static by setting the meassage
      header stamp to ros::Time(0). However, the tf rate shall be set to 30000.
    */
    t.header.stamp = ros::Time(0);  // Current message
    cout << tfBuffer->setTransform(t, ros::this_node::getName(), true) << endl;

    /*
      Check if the specified frame exists in the tree.
    */
    cout << "tfBuffer->_frameExists(map): " << tfBuffer->_frameExists("map") << endl;

    /*
      Get a list strings containing all frames.
    */
    tfBuffer->_getFrameStrings(frames);
    for (auto f : frames)
      cout << "frame: " << f << endl;

    /*
        Get the parent frame of the base_link, return 0 if no parent exist,
        the parent frame name  is stored in the parent string argument.
    */
    cout << tfBuffer->_getParent("base_link", ros::Time(0), parent) << endl;
    cout << parent << endl;

    /*
      Check if a transform is possible from target to source frame at specified time.
      Returns true/false, the error contains information if the target and/or
      source frames are not available.
    */
    errorTF.clear();
    cout << tfBuffer->canTransform("target", "source", ros::Time::now(), &errorTF) << endl;
    cout << errorTF << endl;

    errorTF.clear();
    cout << tfBuffer->canTransform("base_link", "map1", ros::Time::now(), &errorTF) << endl;
    cout << errorTF << endl;

    /*
      Get the tranform from target to source at specified time.
      Make sure to check canTransform before getting the transform.
    */
    cout << tfBuffer->lookupTransform("base_link", "map", ros::Time(0), ros::Duration(10)) << endl;
    cout << errorTF << endl;

    cout << "---" << endl;
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}