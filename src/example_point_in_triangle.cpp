/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include <bits/stdc++.h>
#include <ros/ros.h>

/**
 * @brief Point2D structure to store 2D points (float)
 *
 */

struct Point2D {
  float x, y;
  Point2D(float a = 0, float b = 0) {
    x = a;
    y = b;
  }
  void set(float a, float b) {
    x = a;
    y = b;
  }
  // stream output operator (<<) overloading
  friend std::ostream& operator<<(std::ostream& os, Point2D& pt) {
    os << pt.x << '\t' << pt.y << std::endl;
  }
};

/**
 * @brief Find if a 2D point is within the triangle
 *
 * @param p Point2D to be tested
 * @param p0 Triangle vertex
 * @param p1 Triangle vertex
 * @param p2 Triangle vertex
 * @return true if the point is within the rectangle or on the edge
 * @return false otherwise
 */
bool ptInTriangle(Point2D p, Point2D p0, Point2D p1, Point2D p2) {
  float dX = p.x - p2.x;
  float dY = p.y - p2.y;
  float dX21 = p2.x - p1.x;
  float dY12 = p1.y - p2.y;
  float D = dY12 * (p0.x - p2.x) + dX21 * (p0.y - p2.y);
  float s = dY12 * dX + dX21 * dY;
  float t = (p2.y - p0.y) * dX + (p0.x - p2.x) * dY;
  if (D < 0) return s <= 0 && t <= 0 && s + t >= D;
  return s >= 0 && t >= 0 && s + t <= D;
}

/**
 * @brief Check weather the specified point is within the quadrilateral
 *
 * @param p Point2D to be checked
 * @param p0 First vertex
 * @param p1 Second vertex
 * @param p2 Third vertex
 * @param p3 Fourt  vertex
 * @return true if the point is within the quadrilateral or on the edge
 * @return false otherwise
 */
bool ptInQuadrilateral(Point2D p, Point2D p0, Point2D p1, Point2D p2,
                       Point2D p3) {
  bool result = ptInTriangle(p, p0, p1, p2);
  if (result == false) result = ptInTriangle(p, p0, p1, p2);
  return result;
}

int main(int argc, char* argv[]) {
  // ros::init(argc, argv, "example_point_in_triangle.cpp");
  // ros::NodeHandle nh;

  Point2D p(2, 4.01), p0(0, 0), p1(4, 0), p2(2, 4);
  std::cout.precision(5);
  std::cout << "p:" << p;
  std::cout << "p0:" << p0;
  std::cout << "p1:" << p1;
  std::cout << "p2:" << p2;
  if (ptInTriangle(p, p0, p1, p2)) {
    ROS_INFO("p is inside triangle (p0,p1,p2)");
  } else {
    ROS_INFO("p is outside triangle (p0,p1,p2)");
  }
  return 0;
}