#include <ros/ros.h>

struct Point2D
{
  int x, y;
  Point2D()
  {
    x = y = 0;
  }
  Point2D(int X, int Y)
  {
    x = X, y = Y;
  }
};
/**
 * @brief Bresenham Line drawing algorithm.
 *        Note: http://members.chello.at/~easyfilter/Bresenham.pdf
 *
 * @param ptStart Start point
 * @param ptEnd End point
 * @return std::vector<Point2D>: list of grid cells for the line.
 */
std::vector<Point2D> bresenham(Point2D ptStart, Point2D ptEnd)
{
  std::vector<Point2D> list;
  Point2D pt(ptStart.x, ptStart.y);
  int dx = abs(ptEnd.x - ptStart.x);
  int dy = -abs(ptEnd.y - ptStart.y);
  int sx = (ptStart.x < ptEnd.x) ? 1 : -1;
  int sy = (ptStart.y < ptEnd.y) ? 1 : -1;
  int e = dx + dy, e2;
  while (true)
  {
    list.push_back(pt);
    e2 = 2 * e;
    if (e2 >= dy)
    {
      if (pt.x == ptEnd.x)
        break;
      e += dy;
      pt.x += sx;
    }
    if (e2 <= dx)
    {
      if (pt.y == ptEnd.y)
        break;
      e += dx;
      pt.y += sy;
    }
  }
  return list;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "example_bresenhem_raster_line");
  ros::NodeHandle nh;

  std::vector<Point2D> grid;

  Point2D p1(0, 0), p2(8, 5);  // m = 5/8
  grid = bresenham(p1, p2);
  for (auto&& pt : grid)
    std::cout << pt.x << "," << pt.y << std::endl;
  std::cout << "---" << std::endl;

  Point2D p3(0, 0), p4(5, 8);  // m = 8/5
  grid = bresenham(p3, p4);
  for (auto&& pt : grid)
    std::cout << pt.x << "," << pt.y << std::endl;
  std::cout << "---" << std::endl;

  Point2D p5(0, 0), p6(-5, 8);  // m = -8/5
  grid = bresenham(p5, p6);
  for (auto&& pt : grid)
    std::cout << pt.x << "," << pt.y << std::endl;
  std::cout << "---" << std::endl;

  Point2D p7(0, 0), p8(5, -8);  // m = -8/5
  grid = bresenham(p7, p8);
  for (auto&& pt : grid)
    std::cout << pt.x << "," << pt.y << std::endl;
  std::cout << "---" << std::endl;

  Point2D p9(0, 0), p10(0, 8);  // m = Inf
  grid = bresenham(p9, p10);
  for (auto&& pt : grid)
    std::cout << pt.x << "," << pt.y << std::endl;
  std::cout << "---" << std::endl;

  Point2D p11(0, 0), p12(8, 0);  // m = 0
  grid = bresenham(p11, p12);
  for (auto&& pt : grid)
    std::cout << pt.x << "," << pt.y << std::endl;
  std::cout << "---" << std::endl;

  return 0;
}