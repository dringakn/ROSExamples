#ifndef _OPTIMAL_PATH_PLUGIN_H_
#define _OPTIMAL_PATH_PLUGIN_H_

#include <bits/stdc++.h>
#include <optimal_path_base.h>

namespace optimal_path_plugin {

using namespace optimal_path_base;

class OptimalPath : public OptimaPathBase {
 private:
 public:
  OptimalPath() {}
  ~OptimalPath() {}
  void find(Point pt, std::list<Point> in, std::list<Point>& out) {
    // Start point and previous point variables for the recursive search
    Point current, previous = pt;
    do {
      // Initialize the current point from previous closest point
      current = previous;
      // Add current point to the output list and remove it from the input list
      out.push_back(current), in.remove(current);
      // Find the next nearest point wrt current point.
      double d, minDist = DBL_MAX;
      for (auto&& p : in)
        if (d = current.norm2sq(p), d <= minDist) previous = p, minDist = d;
    } while (in.size());  // Iterate until list is empty
  }
};
};  // namespace optimal_path_plugin

#endif  // !_OPTIMAL_PATH_PLUGIN_H_
