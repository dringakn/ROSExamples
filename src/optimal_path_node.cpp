#include <bits/stdc++.h>
#include <optimal_path_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "optimal_path_node");
  ros::NodeHandle nh;

  // Package name in which the plugin resides.
  pluginlib::ClassLoader<optimal_path_base::OptimaPathBase> loader(
      "ros_examples", "optimal_path_base::OptimalPathBase");
  try {
    boost::shared_ptr<optimal_path_base::OptimaPathBase> path =
        loader.createInstance("optimal_path");

    optimal_path_base::Point pt(0, 0);
    std::list<optimal_path_base::Point> in(10), out;
    std::cout << "Start:" << std::endl << pt << std::endl;

    std::mt19937 rng;
    std::uniform_int_distribution<int> dist(0, 10);

    std::cout << "Input:" << std::endl;
    for (auto&& p : in) {
      p.x = dist(rng), p.y = dist(rng);
      std::cout << p << std::endl;
    }

    path->find(pt, in, out);
    std::cout << "Output:" << std::endl;
    for (auto&& pt : out) std::cout << pt << std::endl;

  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}