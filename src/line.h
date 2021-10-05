#pragma once

#include <math.h>

#include <iostream>
#include <limits>
#include <optional>

#include "vector_2d.h"
#include "vector_3d.h"

namespace geopoly {

template <class T>
inline int sgn(T value) {
  return ((value < T(0)) ? -1 : (value > T(0)) ? 1 : 0);
}

static bool areAlmostEqual(double value1, double value2, double eps = 1e-10) {
  return (std::fabs(value1 - value2) <= eps);
}

class Line {
 public:
  /**
   * Homogeneous representation of line ax + by + c = 0
   **/
  double a, b, c;
  Line() : a(0), b(0), c(0) {}

  Line(const Vector2d<double>& point_a, const Vector2d<double>& point_b) { initializeFromTwoPoints(point_a, point_b); }

  /**
   *     The line is calculated as the cross product of
   *    (point_a.x, point_a.y, 1) and (point_b.x, point_b.y, 1)
   **/
  void initializeFromTwoPoints(const Vector2d<double>& point_a, const Vector2d<double>& point_b) {
    auto crossProduct = cross(point_a, point_b);
    a = crossProduct.x();
    b = crossProduct.y();
    c = crossProduct.z();
  }

  void initializeFromAxisAndPoint(const Vector2d<double>& axis, const Vector2d<double>& point) {
    a = -axis.y();
    b = axis.x();
    c = point.x() * axis.y() - point.y() * axis.x();
  }

  bool areAtSameSide(const Vector2d<double>& point_a, const Vector2d<double>& point_b) const {
    return sgn(a * point_a.x() + b * point_a.y() + c) == sgn(a * point_b.x() + b * point_b.y() + c);
  }

  bool isPointOnLine(const Vector2d<double>& point) const {
    return areAlmostEqual(a * point.x() + b * point.y() + c, 0.0);
  }

  void normalize() {
    double sqrtCoeffs = sqrt(a * a + b * b + c * c);
    a /= sqrtCoeffs;
    b /= sqrtCoeffs;
    c /= sqrtCoeffs;
  }
};

class LineSegment {
 public:
  LineSegment(const Vector2d<double>& a, const Vector2d<double>& b) : point_a(a), point_b(b) { assignLimits(); }

  bool intersectsWithHorizontalLine(double y) const { return (min.y() <= y && y <= max.y()); }

  bool isInRange(const Vector2d<double>& point) const {
    if ((min.y() <= point.y() && point.y() <= max.y()) && (min.x() <= point.x() && point.x() <= max.x())) return true;
    return false;
  }

  Vector2d<double> point_a;
  Vector2d<double> point_b;

 private:
  void assignLimits() {
    min.x() = (point_a.x() > point_b.x()) ? point_b.x() : point_a.x();
    min.y() = (point_a.y() > point_b.y()) ? point_b.y() : point_a.y();
    max.x() = (point_a.x() < point_b.x()) ? point_b.x() : point_a.x();
    max.y() = (point_a.y() < point_b.y()) ? point_b.y() : point_a.y();
  }

  Vector2d<double> min = Vector2d<double>({std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});
  Vector2d<double> max = Vector2d<double>({-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()});
};

class HalfLineSegment {
 public:
  HalfLineSegment(const Vector2d<double>& axis_, const Vector2d<double>& start_) : start(start_) {
    axis = axis_ / axis_.length();
    line.initializeFromAxisAndPoint(axis, start);
  }

  bool isPointOnLine(const Vector2d<double>& point) const {
    if (!line.isPointOnLine(point)) return false;
    Vector2d<double> newAxis = point - start;
    newAxis = newAxis / newAxis.length();
    return areAlmostEqual(newAxis, axis, 1e-5);
  }

  Line line;
  Vector2d<double> start;
  Vector2d<double> axis;
};

inline bool operator==(const LineSegment& line1, const LineSegment& line2) {
  return areAlmostEqual(line1.point_a.x(), line2.point_a.x()) && areAlmostEqual(line1.point_b.y(), line2.point_b.y());
}

inline bool operator!=(const LineSegment& line1, const LineSegment& line2) { return !(line1 == line2); }

static Line toLine(const LineSegment& lineSegment) { return Line(lineSegment.point_a, lineSegment.point_b); }

static std::optional<Vector2d<double>> computeIntersection(const Line& line1, const Line& line2) {
  Vector3d<double> line_1_vec{line1.a, line1.b, line1.c};
  Vector3d<double> line_2_vec{line2.a, line2.b, line2.c};
  Vector3d<double> intersection = cross(line_1_vec, line_2_vec);

  if (areAlmostEqual(intersection.z(), 0.0)) {
    return {};
  }

  return Vector2d<double>({intersection.x() / intersection.z(), intersection.y() / intersection.z()});
}

static std::optional<Vector2d<double>> computeIntersection(const LineSegment& lineSegment1,
                                                           const LineSegment& lineSegment2) {
  auto line1 = toLine(lineSegment1);
  if (line1.areAtSameSide(lineSegment2.point_a, lineSegment2.point_b)) {
    return {};
  }
  auto line2 = toLine(lineSegment2);
  auto intersection = computeIntersection(line1, line2);
  if (!intersection) return {};
  if (lineSegment1.isInRange(intersection.value()) && lineSegment1.isInRange(intersection.value())) return intersection;
  return {};
}

static std::optional<Vector2d<double>> computeIntersection(const LineSegment& lineSegment,
                                                           const HalfLineSegment& halfLineSegment) {
  auto line1 = toLine(lineSegment);
  auto& line2 = halfLineSegment.line;
  auto intersection = computeIntersection(line1, line2);
  if (!intersection) return {};
  if (halfLineSegment.isPointOnLine(intersection.value())) return intersection;
  return {};
}

static std::optional<Vector2d<double>> computeIntersection(const HalfLineSegment& halfLineSegment,
                                                           const LineSegment& lineSegment) {
  return computeIntersection(lineSegment, halfLineSegment);
}
}  // namespace geopoly
