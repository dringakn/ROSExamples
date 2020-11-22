/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
    Notes:
        Nodelets are designed to provide a way to run multiple algorithms on a
        single machine, in a single process, without incurring copy costs when
        passing messages intraprocess.
        The primary advantage is the automagic
        zero-copy transport between nodelets (in one nodelet manager). This
        means that the pointcloud created by a hardware driver doesn’t need to
        get copied or serialized before it hits your code, assuming you inject
        the nodelet into the camera’s manager, saving you time and trouble.

        You get all the modularity of nodes, and
        all the efficiency of having one monolithic process. This makes nodelets
        more flexible than bare plugins (via pluginlib) - you can implicitly tap
        into any of the intra-process communication that occurs.

        The requirement for zero-copy transport to work is that you subscribe
        with a ConstPtr callback, and don’t modify the message on the publisher
        side after publishing, see nodelet code below.

        The earliest you can get a NodeHandle is inside the onInit()
        method. Don’t try to do anything ROS related in the constructor. If one
        nodelet goes down, the whole manager goes down. Check for exceptions.

        To obey the Nodelet API, you shouldn’t manually manage threads.
        But you were always using ros::Timer callbacks like you’re supposed to
        anyways, right? Callbacks on NodeHandles from the Nodelet API get
        managed by a shared threadpool, which is way more efficient.

        ROS_DEBUG and friends no longer work - use the equivalent
        NODELET_DEBUG. Sadly this precludes sharing code implementation with
        debug messages between Nodes and Nodelets, which is why we use a dynamic
        wrapping Node below.
*/
#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_nodelet_node");
  ros::NodeHandle nh;
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "Package/Nodelet", remap, nargv);
  ros::spin();
  return 0;
}