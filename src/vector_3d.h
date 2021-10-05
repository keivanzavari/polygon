#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>

#include "types.h"
#include "vector_2d.h"
#include "vector_nd.h"

namespace geopoly {

template <typename Type>
using Vector3d = VectorNd<Type, 3>;

template <typename Type>
static Vector3d<Type> cross(const Vector3d<Type>& vector1, const Vector3d<Type>& vector2) {
  Type x = vector1.y() * vector2.z() - vector1.z() * vector2.y();
  Type y = vector1.z() * vector2.x() - vector1.x() * vector2.z();
  Type z = vector1.x() * vector2.y() - vector1.y() * vector2.x();
  return Vector3d<Type>{x, y, z};
}

template <typename Type>
static Vector3d<Type> cross(const Vector2d<Type>& vector1, const Vector2d<Type>& vector2) {
  Type x = vector1.y() - vector2.y();
  Type y = vector2.x() - vector1.x();
  Type z = vector1.x() * vector2.y() - vector1.y() * vector2.x();
  return Vector3d<Type>{x, y, z};
}

}  // namespace geopoly
