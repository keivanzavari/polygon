#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "line.h"
#include "vector_2d.h"

namespace geopoly {

int getDirectionFromThree(const Vector2d<double>& point1, const Vector2d<double>& point2,
                          const Vector2d<double>& point3) {
  auto vec1 = point2 - point1;
  auto vec2 = point3 - point2;
  auto res = cross(vec1, vec2);
  int direction = 0;
  if (areAlmostEqual(res.z(), 0))
    direction = 0;
  else if (res.z() > 0)
    direction = 1;
  else
    direction = -1;
  return direction;
}

class Polygon {
 public:
  Polygon(const std::vector<Vector2d<double>>& vertices_) : vertices(vertices_) {
    if (vertices.size() <= 2) {
      throw std::invalid_argument("Polygon should at least have 3 vertices.");
    }
    assignLimits();
    computeEdges();
  }

  bool contains(const Vector2d<double>& point) const {
    if (isOutsideLimits(point)) {
      std::cout << "outside limits\n";
      return false;
    }

    HalfLineSegment halfLineSegment({1, 0}, point);
    int count_intersections = 0;
    for (const auto& edge : edges) {
      if (edge.intersectsWithHorizontalLine(point.y()) && computeIntersection(edge, halfLineSegment)) {
        count_intersections++;
      }
    }
    std::cout << "num intersectinos: " << count_intersections << "\n";

    if (count_intersections % 2 == 0) {
      return false;
    }
    return true;
  }

  const std::vector<LineSegment>& getEdges() const { return edges; }

  bool isSelfIntersecting() const {
    // The edges should only intersect at vertices
    for (const auto& edge1 : edges) {
      for (const auto& edge2 : edges) {
        if (edge1 != edge2) {
          const auto intersection = computeIntersection(edge1, edge2);
          if (intersection && !isVertex(intersection.value())) {
            return true;
          }
        }
      }
    }

    return false;
  }

  bool isConvex() const {
    // find out if going from one vertex to another requires left or right turn.
    // If they are all the same, it is convex.
    // The number of angles is the same as the number of vertices. In some self intersecting shapes like a star,
    // the direction doesn't change.
    if (vertices.size() == 3) return true;
    int initDirection = getDirectionFromThree(vertices[0], vertices[1], vertices[2]);

    // int size = static_cast<int>(vertices.size());
    for (std::size_t idx = 1; idx < vertices.size(); ++idx) {
      int idx1 = (idx + 1) % vertices.size();
      int idx2 = (idx + 2) % vertices.size();
      int currDirection = getDirectionFromThree(vertices[idx], vertices[idx1], vertices[idx2]);
      if (currDirection != initDirection) return false;
    }

    // If it is convex so far, check for self intersection.
    return !isSelfIntersecting();
  }

 private:
  void assignLimits() {
    for (const auto& vertex : vertices) {
      if (vertex.x() < min.x()) {
        min.x() = vertex.x();
      }
      if (vertex.x() > max.x()) {
        max.x() = vertex.x();
      }
      if (vertex.y() < min.y()) {
        min.y() = vertex.y();
      }
      if (vertex.y() > max.y()) {
        max.y() = vertex.y();
      }
    }
  }

  void computeEdges() {
    edges.reserve(vertices.size());
    for (auto it = vertices.begin(); it != vertices.end() - 1; ++it) {
      edges.emplace_back(*it, *std::next(it));
    }
    edges.emplace_back(vertices.back(), vertices.front());
  }

  bool isOutsideLimits(const Vector2d<double>& point) const {
    return (point.x() > max.x() || point.x() < min.x() || point.y() < min.y() || point.y() > max.y());
  }

  bool isVertex(const Vector2d<double>& point) const {
    const auto it = std::find_if(vertices.begin(), vertices.end(), [&](const Vector2d<double>& vertex) {
      return areAlmostEqual(vertex.x(), point.x()) && areAlmostEqual(vertex.y(), point.y());
    });

    return (it != vertices.end());
  }

  std::vector<Vector2d<double>> vertices;
  std::vector<LineSegment> edges;
  Vector2d<double> min = Vector2d<double>({std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});
  Vector2d<double> max = Vector2d<double>({-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()});
};
}  // namespace geopoly
