#include "polygon.h"

#include <gtest/gtest.h>
namespace geopoly {

TEST(Polygon, Construct) {
  std::vector<Vector2d<double>> vertices = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};

  auto polygon = Polygon(vertices);
  const auto edges = polygon.getEdges();
  ASSERT_EQ(edges.size(), 4);
}

TEST(Polygon, Contains) {
  std::vector<Vector2d<double>> vertices = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};

  auto polygon = Polygon(vertices);
  ASSERT_TRUE(polygon.contains({0.5, 0.5}));
}

}  // namespace geopoly
