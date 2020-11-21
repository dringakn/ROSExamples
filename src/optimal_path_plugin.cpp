#include <optimal_path_base.h>
#include <optimal_path_plugin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(optimal_path_plugin::OptimalPath,
                       optimal_path_base::OptimaPathBase)