/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        - This example demonstrates how to dynamically reconfigure ros
          parameters. As an example a few sample parameters are defined.
          The parameters values can be changed using rqt_reconfigure.
          rosrun rqt_reconfigure rqt_reconfigure.
          The node will be notified as soon as the parameters are changed.

    Notes:
        - Add dynamic_reconfigure as package dependency.
        - Add it also to the package.xml (build_depend and exec_depend)
        - Inside the CMakeLists.txt the node should be linked with
          add_dependencies(node_example ${PROJECT_NAME}_gencfg)
        - Create a cfg foler and create a file MyParams.cfg inside it.
          Set it’s attributes chmod a+x.
          Example cfg file:
          #!/usr/bin/env python
          from dynamic_reconfigure.parameter_generator_catkin import *
          PACKAGE = "node_example"

          gen = ParameterGenerator()
          # (Name, ParamType, Level, Description, Default, Min, Max)
          gen.add("rate", double_t, 0, "Message rate (mSec)", 1, 0.001, 100)
          gen.add("enable", bool_t, 0, "Enable/Disable publisher", True)
          gen.add("message", str_t, 0, "Message to be published", "Bello!")

          my_enum = gen.enum([gen.const("Small", int_t, 0, "Small Constant"),
                              gen.const("Medium", int_t, 1, "Medium Constant"),
                              gen.const("Large", int_t, 2, "Large Constant"),
                              gen.const("ExtraLarge", int_t, 3, "Extra Large
                                        Constant")], "Options")

          gen.add("choice", int_t, 0, "Select option", 3, 0, 3,
                    edit_method=my_enum)

          exit(gen.generate(PACKAGE, "node_example", "MyParams"))

        - Uncomment the generate dynamic reconfigure options
          generate_dynamic_reconfigure_options(cfg/MyParams.cfg)
        - Make sure the Python code formatting extension is installed. It may
            require sudo apt install python3-pip
        - The server node which listens the parameter changes by the
            rqt_reconfigure node is as follows in this example.
        - The ExampleConfig file is within the include directory of development workspace!!!!
*/
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros_examples/ExampleConfig.h>
// ~/{name}_ws/devel/include/ros_examples/ExampleConfig.h

/**
 * @brief Parameters update callback
 *
 * @param config updated parameters structure
 * @param level
 */
void callback(ros_examples::ExampleConfig& config, uint32_t level) {
  ROS_INFO("Reconfigure Request [%d]: %d %f %s %s", level, config.int_param,
           config.double_param, config.param.c_str(),
           config.bool_param ? "True" : "False");
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_dynamic_reconfigure_rosparam");
  ros::NodeHandle nh;

  // Create dynamic reconfigure server
  dynamic_reconfigure::Server<ros_examples::ExampleConfig> server;

  // Create a callback object and assign the function to be called
  dynamic_reconfigure::Server<ros_examples::ExampleConfig>::CallbackType cb;
  cb = boost::bind(&callback, _1, _2);
  server.setCallback(cb);

  ros::spin();
  return 0;
}