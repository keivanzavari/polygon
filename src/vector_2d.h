#pragma once
#include <array>
#include <cmath>

#include "types.h"
#include "vector_nd.h"

namespace geopoly {

template <typename Type>
using Vector2d = VectorNd<Type, 2>;

template <typename Type>
static AngleRad getAngle(const Vector2d<Type>& vec) {
  return AngleRad(atan2(vec[1], vec[0]));
}

template <typename Type>
bool areAlmostEqual(const Vector2d<Type>& v1, const Vector2d<Type>& v2, double tolerance = 1e-10) {
  return (v1 - v2).length() <= tolerance;
}

}  // namespace geopoly
