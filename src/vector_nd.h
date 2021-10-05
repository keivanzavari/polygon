#pragma once

#include <array>
namespace geopoly {

template <typename Type, std::size_t kNumElements>
class VectorNd {
 public:
  constexpr VectorNd() = default;
  constexpr VectorNd(const std::array<Type, kNumElements>& other) : elements(other) {}
  constexpr VectorNd(const std::initializer_list<Type>& other) {
    auto it = other.begin();
    for (auto& e : elements) {
      e = *it;
      ++it;
    }
  }

  constexpr VectorNd<Type, kNumElements>& operator=(const VectorNd<Type, kNumElements>& other) {
    elements = other.elements;
    return *this;
  }

  Type& operator[](unsigned int index) { return elements[index]; }
  Type const& operator[](unsigned int index) const { return elements[index]; }

  auto begin() { return elements.begin(); }
  auto end() { return elements.end(); }
  auto begin() const { return elements.cbegin(); }
  auto end() const { return elements.cend(); }

  VectorNd<Type, kNumElements>& operator+=(const VectorNd<Type, kNumElements>& rhs) {
    for (unsigned int i = 0; i < elements.size(); ++i) {
      elements[i] += rhs[i];
    }
    return *this;
  }

  VectorNd<Type, kNumElements>& operator-=(const VectorNd<Type, kNumElements>& rhs) {
    for (unsigned int i = 0; i < elements.size(); ++i) {
      elements[i] -= rhs[i];
    }
    return *this;
  }

  template <class Scalar>
  VectorNd<Type, kNumElements>& operator*=(const Scalar& scalar) {
    for (unsigned int i = 0; i < elements.size(); ++i) {
      elements[i] *= scalar;
    }
    return *this;
  }

  auto dot(const VectorNd<Type, kNumElements>& v) const -> decltype(Type() * Type()) {
    auto result = Type() * Type();
    for (unsigned int i = 0; i < elements.size(); ++i) {
      result += elements[i] * v[i];
    }
    return result;
  }

  Type length() const { return sqrt(dot(*this)); }

  void fill(Type element) {
    for (auto& e : elements) {
      e = element;
    }
  }

  Type x() const { return elements[0]; }
  Type& x() { return elements[0]; }
  Type y() const { return elements[1]; }
  Type& y() { return elements[1]; }
  Type z() const { return elements[2]; }
  Type& z() { return elements[2]; }

 private:
  std::array<Type, kNumElements> elements;
};

template <typename Type, std::size_t kNumElements>
VectorNd<Type, kNumElements> operator+(const VectorNd<Type, kNumElements>& v1, const VectorNd<Type, kNumElements>& v2) {
  VectorNd<Type, kNumElements> res;
  for (unsigned int i = 0; i < kNumElements; ++i) {
    res[i] = v1[i] + v2[i];
  }
  return res;
}

template <typename Type, std::size_t kNumElements>
VectorNd<Type, kNumElements> operator-(const VectorNd<Type, kNumElements>& v1, const VectorNd<Type, kNumElements>& v2) {
  VectorNd<Type, kNumElements> res;
  for (unsigned int i = 0; i < kNumElements; ++i) {
    res[i] = v1[i] - v2[i];
  }
  return res;
}

template <typename Type, std::size_t kNumElements>
VectorNd<Type, kNumElements> operator+(const VectorNd<Type, kNumElements>& v1, const Type& scalar) {
  VectorNd<Type, kNumElements> res;
  for (unsigned int i = 0; i < kNumElements; ++i) {
    res[i] = v1[i] + scalar;
  }
  return res;
}

template <typename Type, std::size_t kNumElements, typename Scalar>
VectorNd<Type, kNumElements> operator*(const VectorNd<Type, kNumElements>& v1, const Scalar& scalar) {
  VectorNd<Type, kNumElements> res;
  for (unsigned int i = 0; i < kNumElements; ++i) {
    res[i] = v1[i] * scalar;
  }
  return res;
}

template <typename Type, std::size_t kNumElements, typename Scalar>
VectorNd<Type, kNumElements> operator/(const VectorNd<Type, kNumElements>& v1, const Scalar& scalar) {
  VectorNd<Type, kNumElements> res;
  for (unsigned int i = 0; i < kNumElements; ++i) {
    res[i] = v1[i] / scalar;
  }
  return res;
}

template <typename Type, std::size_t kNumElements>
VectorNd<Type, kNumElements> operator*(const Type& scalar, const VectorNd<Type, kNumElements>& vector) {
  return vector * scalar;
}

template <typename Type, std::size_t kNumElements>
std::ostream& operator<<(std::ostream& os, const VectorNd<Type, kNumElements>& vec) {
  os << "[";
  for (const auto& v : vec) os << v << " ";
  os << "]";
  return os;
}
}  // namespace geopoly
