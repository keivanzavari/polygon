#include "line.h"

#include <gtest/gtest.h>
namespace geopoly {

TEST(Line, ConstructTest1) {
  Line line({4, -2}, {3, 4});

  ASSERT_DOUBLE_EQ(line.a, -6);
  ASSERT_DOUBLE_EQ(line.b, -1);
  ASSERT_DOUBLE_EQ(line.c, 22);
}

TEST(Line, ConstructTest2) {
  Line line({4, 5}, {0, 5});

  ASSERT_DOUBLE_EQ(line.a, 0);
  ASSERT_DOUBLE_EQ(line.b, -4);
  ASSERT_DOUBLE_EQ(line.c, 20);

  line.normalize();

  ASSERT_DOUBLE_EQ(line.a, 0);
  ASSERT_DOUBLE_EQ(line.b, -4 / sqrt(416));
  ASSERT_DOUBLE_EQ(line.c, 20 / sqrt(416));
}

TEST(Line, ConstructTest3) {
  Line line({2.5, 0}, {2.5, 4});

  ASSERT_DOUBLE_EQ(line.a, -4);
  ASSERT_DOUBLE_EQ(line.b, 0);
  ASSERT_DOUBLE_EQ(line.c, 10);

  line.normalize();
  ASSERT_DOUBLE_EQ(line.a, -4 / sqrt(116));
  ASSERT_DOUBLE_EQ(line.b, 0);
  ASSERT_DOUBLE_EQ(line.c, 10 / sqrt(116));
}

TEST(Line, SideTest1) {
  Vector2d<double> point1{1, 3};
  Vector2d<double> point2{2, 7};
  Line line1(point1, point2);

  ASSERT_TRUE(line1.areAtSameSide(Vector2d<double>{1, 4}, Vector2d<double>{1.5, 7}));
  ASSERT_TRUE(line1.isPointOnLine(point1));
  ASSERT_TRUE(line1.isPointOnLine(point2));

  Vector2d<double> point3{2, 3};
  Vector2d<double> point4{1, 6};
  Line line2(point3, point4);

  ASSERT_FALSE(line2.areAtSameSide(Vector2d<double>{1, 4}, Vector2d<double>{1.5, 7}));

  ASSERT_TRUE(line2.isPointOnLine(point3));
  ASSERT_TRUE(line2.isPointOnLine(point4));
}

TEST(Line, SideTest2) {
  Vector2d<double> point1{0, 0};
  Vector2d<double> point2{5, 0};
  Line line1(point1, point2);

  Vector2d<double> point3{-4, 3};
  Vector2d<double> point4{-4, -6};
  Line line2(point3, point4);

  ASSERT_FALSE(line1.areAtSameSide(point3, point4));
  ASSERT_TRUE(line2.areAtSameSide(point1, point2));
}

TEST(LineSegment, ConstructTest) {
  Vector2d<double> point1{5.0, 0.0};
  Vector2d<double> point2{0.0, 0.0};

  LineSegment lineSegment(point1, point2);
  ASSERT_FALSE(lineSegment.intersectsWithHorizontalLine(1.0));
  ASSERT_FALSE(lineSegment.intersectsWithHorizontalLine(-1.0));
  ASSERT_TRUE(lineSegment.intersectsWithHorizontalLine(0.0));
}

TEST(LineSegment, IsInRange1) {
  Vector2d<double> point1{5.0, 0.0};
  Vector2d<double> point2{0.0, 0.0};

  LineSegment lineSegment(point1, point2);
  ASSERT_FALSE(lineSegment.isInRange({1.0, 1.0}));
}

TEST(LineSegment, IsInRange2) {
  Vector2d<double> point1{0.0, 0.0};
  Vector2d<double> point2{1.0, 1.0};

  LineSegment lineSegment(point1, point2);
  ASSERT_TRUE(lineSegment.isInRange(point1));
  ASSERT_TRUE(lineSegment.isInRange(point2));
  ASSERT_TRUE(lineSegment.isInRange({0.5, 0.5}));
  ASSERT_FALSE(lineSegment.isInRange({-0.5, 0.5}));
  ASSERT_FALSE(lineSegment.isInRange({0.5, -0.5}));
}

TEST(LineSegment, IntersectionTest1) {
  Vector2d<double> point1{5.0, 0.0};
  Vector2d<double> point2{0.0, 0.0};
  LineSegment lineSegment1(point1, point2);

  Vector2d<double> point3{5.0, 20.0};
  Vector2d<double> point4{0.0, 20.0};
  LineSegment lineSegment2(point1, point2);

  auto intersection = computeIntersection(lineSegment1, lineSegment2);

  ASSERT_FALSE(intersection.has_value());
}

TEST(LineSegment, IntersectionTest2) {
  Vector2d<double> point1{5.0, 0.0};
  Vector2d<double> point2{0.0, 0.0};
  LineSegment lineSegment1(point1, point2);

  Vector2d<double> point3{-2.0, 2.0};
  Vector2d<double> point4{-2.0, -3.0};
  LineSegment lineSegment2(point3, point4);

  // The two lines intersect, but the line segments do not.
  auto intersection = computeIntersection(lineSegment1, lineSegment2);
  ASSERT_FALSE(intersection.has_value());
  auto line1 = toLine(lineSegment1);
  auto line2 = toLine(lineSegment2);

  intersection = computeIntersection(line1, line2);
  ASSERT_TRUE(intersection.has_value());
}

TEST(Line, IntersectionTest1) {
  Vector2d<double> point1{1, 1};
  Vector2d<double> point2{2, 2};
  Line line1(point1, point2);

  Vector2d<double> point3{0, 0};
  Vector2d<double> point4{0, 1};
  Line line2(point3, point4);

  auto intersection = computeIntersection(line1, line2);

  ASSERT_TRUE(intersection.has_value());
  ASSERT_DOUBLE_EQ(intersection.value().x(), 0.0);
  ASSERT_DOUBLE_EQ(intersection.value().y(), 0.0);
}

TEST(Line, IntersectionTest2) {
  Vector2d<double> point1{1, 3};
  Vector2d<double> point2{2, 7};
  Line line1(point1, point2);

  Vector2d<double> point3{2, 3};
  Vector2d<double> point4{1, 6};
  Line line2(point3, point4);

  auto intersection = computeIntersection(line1, line2);

  ASSERT_TRUE(intersection.has_value());
  ASSERT_DOUBLE_EQ(intersection.value().x(), 10.0 / 7.0);
  ASSERT_DOUBLE_EQ(intersection.value().y(), 33.0 / 7.0);
}

}  // namespace geopoly
