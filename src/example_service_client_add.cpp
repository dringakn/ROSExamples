/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Desciption: Client for addition request from the server node
 *                (example_service_server_add).
 *                filename: addsrv.srv
 *
 *                float64 a
 *                float64 b
 *                ---
 *                float64 result
 *    Notes:
 *          CMakeLists.txt:
 *          add_service_files(... addsrv.srv ...)
 *          find_package(catkin REQUIRED COMPONENTS ... message_generation ... )
 *          add_service_files(FILES ... mysrv.srv ...)
 *          generate_messages(DEPENDENCIES ... name_of_package_used_in_srv ...)
 *          catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)
 *          Package.xml:
 *          <build_depend>message_generation</build_depend>
 *          <build_export_depend>message_generation</build_export_depend>
 *          <exec_depend>message_runtime</exec_depend>
 **/
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include <ros_examples/addsrv.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_service_client_add");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  random_numbers::RandomNumberGenerator rng;
  ros::ServiceClient client = nh.serviceClient<ros_examples::addsrv>("addservice");

  // Create a service client to connect to octomap_server on "octomap_binary"
  // client.call(req,res) // actual call to the service
  // client.exists() // true if advertised and available
  // client.getService() // return the name of the service
  // client.isPersistent() // persistent service
  // client.isValid() // handle for persistent service
  // client.waitForExistence(ros::Duration(-1)) // blocked wait

  while (ros::ok())
  {
    ros_examples::addsrv srv;
    srv.request.a = rng.uniformInteger(0, 99);
    srv.request.b = rng.uniformInteger(0, 99);
    if (client.call(srv))
    {
      ROS_INFO("Sum:%f", srv.response.result);
    }
    else
    {
      ROS_INFO("Failed to call service");
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}