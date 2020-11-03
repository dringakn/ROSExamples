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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_tf2_listener2");
  ros::NodeHandle nh("~");

  // Create a TF buffer object
  tfBuffer = new tf2_ros::Buffer(ros::Duration(10), false);
  // Create a TF listener object to fill buffer
  tf2_ros::TransformListener li(*tfBuffer, nh, true);
  // error message string for canTransform and lookup transform
  string errorTF;
  // vector of strings (frames)
  ros::V_string frames;
  // parent frame query
  string parent;

  ros::Rate rate(1);
  while (ros::ok())
  {
    /*
      Returns a string with list of cached frames, useful for debugging.
      For example the output is as follows:
        Frame sensor2 exists with parent base_link.
        Frame base_link exists with parent map.
        Frame sensor exists with parent base_link.
    */
    cout << tfBuffer->allFramesAsString() << endl;

    /*
      Extra information for debugging.
      For example the output is as follows:
        sensor2:
          parent: 'base_link'
          broadcaster: '/example_tf2_listener'
          rate: 10000.000
          most_recent_transform: 0.000
          oldest_transform: 0.000
          buffer_length: 0.000
        base_link:
          parent: 'map'
          broadcaster: 'unknown_publisher'
          rate: 10.084
          most_recent_transform: 1604362575.879
          oldest_transform: 1604362566.260
          buffer_length: 9.619

    */
    cout << tfBuffer->allFramesAsYAML() << endl;

    /*
      Return the current cached length time.
    */
    cout << tfBuffer->getCacheLength().toSec() << endl;

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