#ifndef _OPTIMAL_PATH_BASE_H_
#define _OPTIMAL_PATH_BASE_H_

#include <bits/stdc++.h>

namespace optimal_path_base {
struct Point {
  double x, y;
  Point() { x = y = 0; }
  Point(double X, double Y) { x = X, y = Y; }
  inline double norm1(Point p) { return abs(x - p.x) + abs(y - p.y); }
  inline double norm2(Point p) { return sqrt(norm2sq(p)); }
  inline double norm2sq(Point p) { return pow(x - p.x, 2) + pow(y - p.y, 2); }
  Point operator+(const Point& p) { return Point(x + p.x, y + p.y); }
  Point operator-(const Point& p) { return Point(x - p.x, y - p.y); }
  friend inline bool operator<(const Point& lhs, const Point& rhs) {
    return (abs(lhs.x + lhs.y) < abs(rhs.x + rhs.y));
    // return ((lhs.x * lhs.x + lhs.y * lhs.y) <
    //         abs(rhs.x * rhs.x + rhs.y * rhs.y));
  }
  friend inline bool operator>(const Point& lhs, const Point& rhs) {
    return rhs < lhs;
  }
  friend inline bool operator==(const Point& lhs, const Point& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y);
  }
  friend inline bool operator!=(const Point& lhs, const Point& rhs) {
    return !(lhs == rhs);
  }
  friend std::ostream& operator<<(std::ostream& os, const Point& rhs) {
    os << "(" << rhs.x << "," << rhs.y << ")";
    return os;
  }
};

class OptimaPathBase {
 public:
  virtual void find(Point pt, std::list<Point> in, std::list<Point>& out) = 0;
};
};  // namespace optimal_path_base

#endif  // !_OPTIMAL_PATH_BASE_H_
