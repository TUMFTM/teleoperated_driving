// Copyright 1999 KJIST CAD/CAM     Ryu, Jae Hun ( ryu@geguri.kjist.ac.kr)
// Circle.h: interface for the Circle class.
// Circle class.
// Purpose : Represent the circle object
// Input : 3 different points
// Process : Calcuate the radius and center
// Output : Circle
//
// This class originally designed for representation of discretized curvature information
// of sequential pointlist
// KJIST CAD/CAM     Ryu, Jae Hun ( ryu@geguri.kjist.ac.kr)
// Last update : 1999. 7. 4
#ifndef TOD_HELPER__GEOMETRY__CIRCLE_H_
#define TOD_HELPER__GEOMETRY__CIRCLE_H_

#include "rclcpp/rclcpp.hpp"
namespace tod_helper::Geometry
{
class Point
{
public:
  Point(double x, double y, double z)
  {
    m_x = x;
    m_y = y;
    m_z = z;
  };
  Point() = default;
  double m_x{0.0}, m_y{0.0}, m_z{0.0};
  double x() const { return m_x; }
  double y() const { return m_y; }
  double z() const { return m_z; }
};
class Circle
{
public:
  double GetRadius();
  double GetCurvature();
  double getOrientationOfTangentInPoint(const Point & pt);
  bool isValidCircle();
  Point GetCenter();
  Circle(Point & p1, Point & p2, Point & p3);  // p1, p2, p3 are co-planar
  Circle();
  virtual ~Circle() = default;

private:
  double CalcCircle(Point & pt1, Point & pt2, Point & pt3);
  double m_dRadius;
  Point m_Center;
};
inline double get_distance_between_points(Point & point1, Point & point2)
{
  return std::sqrt(std::pow(point1.x() - point2.x(), 2) + std::pow(point1.y() - point2.y(), 2));
}
inline double get_orientation_of_line(Point & point1, Point & point2)
{
  double dx = point2.x() - point1.x();
  double dy = point2.y() - point1.y();
  double yaw1 = std::atan2(dy, dx);
  return yaw1;
}
inline bool isLeft(Point line1, Point line2, Point pt)
{
  return ((line2.x() - line1.x()) * (pt.y() - line1.y()) -
          (line2.y() - line1.y()) * (pt.x() - line1.x())) > 0;
}
};  // namespace tod_helper::Geometry

#endif  // TOD_HELPER__GEOMETRY__CIRCLE_H_
