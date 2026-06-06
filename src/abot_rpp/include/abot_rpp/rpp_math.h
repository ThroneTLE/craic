/**
 * @file: rpp_math.h
 * @brief: Minimal math/geometry utilities for RPP controller
 *         (extracted from ros_motion_planning/common)
 */
#ifndef ABOT_RPP_MATH_H_
#define ABOT_RPP_MATH_H_

#include <cmath>
#include <vector>
#include <algorithm>

namespace abot_rpp {

// ---- Vec2d ----
class Vec2d {
public:
  constexpr Vec2d(double x, double y) : x_(x), y_(y) {}
  constexpr Vec2d() : Vec2d(0, 0) {}

  double x() const { return x_; }
  double y() const { return y_; }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }

  double length() const { return std::hypot(x_, y_); }
  double lengthSquare() const { return x_ * x_ + y_ * y_; }
  double angle() const { return std::atan2(y_, x_); }

  double distanceTo(const Vec2d& o) const { return (*this - o).length(); }

  double crossProd(const Vec2d& o) const { return x_ * o.y_ - y_ * o.x_; }
  double innerProd(const Vec2d& o) const { return x_ * o.x_ + y_ * o.y_; }

  Vec2d rotate(double angle) const {
    double c = std::cos(angle), s = std::sin(angle);
    return Vec2d(x_ * c - y_ * s, x_ * s + y_ * c);
  }

  Vec2d operator+(const Vec2d& o) const { return Vec2d(x_ + o.x_, y_ + o.y_); }
  Vec2d operator-(const Vec2d& o) const { return Vec2d(x_ - o.x_, y_ - o.y_); }
  Vec2d operator*(double r) const { return Vec2d(x_ * r, y_ * r); }
  Vec2d operator/(double r) const { return Vec2d(x_ / r, y_ / r); }
  Vec2d& operator+=(const Vec2d& o) { x_ += o.x_; y_ += o.y_; return *this; }
  Vec2d& operator-=(const Vec2d& o) { x_ -= o.x_; y_ -= o.y_; return *this; }
  Vec2d& operator*=(double r) { x_ *= r; y_ *= r; return *this; }
  bool operator==(const Vec2d& o) const { return x_ == o.x_ && y_ == o.y_; }

protected:
  double x_ = 0.0, y_ = 0.0;
};

// ---- Point3d ----
class Point3d {
public:
  Point3d(double x = 0, double y = 0, double theta = 0) : x_(x), y_(y), theta_(theta) {}
  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setTheta(double t) { theta_ = t; }
private:
  double x_, y_, theta_;
};

using Points3d = std::vector<Point3d>;

// ---- Angle utilities ----
inline double normalizeAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  return (a <= 0.0) ? a + M_PI : a - M_PI;
}

// ---- Clamp ----
template <typename T>
T clamp(T v, T lo, T hi) {
  if (lo > hi) std::swap(lo, hi);
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---- Circle-segment intersection ----
inline std::vector<Vec2d> circleSegmentIntersection(
    const Vec2d& p1, const Vec2d& p2, double r) {
  std::vector<Vec2d> result;
  double dx = p2.x() - p1.x();
  double dy = p2.y() - p1.y();
  double dr2 = dx * dx + dy * dy;
  double D = p1.x() * p2.y() - p2.x() * p1.y();

  double disc = r * r * dr2 - D * D;
  if (disc < 0) return result;

  double sgn = (dy < 0) ? -1.0 : 1.0;
  double sqrt_disc = std::sqrt(disc);

  double x1 = ( D * dy + sgn * dx * sqrt_disc) / dr2;
  double y1 = (-D * dx + std::abs(dy) * sqrt_disc) / dr2;
  result.push_back(Vec2d(x1, y1));

  if (disc > 1e-10) {
    double x2 = ( D * dy - sgn * dx * sqrt_disc) / dr2;
    double y2 = (-D * dx - std::abs(dy) * sqrt_disc) / dr2;
    result.push_back(Vec2d(x2, y2));
  }
  return result;
}

// ---- Arc center (curvature from 3 points) ----
inline double arcCenter(const Vec2d& pt_prev, const Vec2d& pt,
                         const Vec2d& pt_next, bool is_cusp,
                         Vec2d* center = nullptr) {
  if (is_cusp) return 0.0;

  Vec2d p1 = pt_prev, p2 = pt, p3 = pt_next;
  double dx2 = p2.x() - p1.x(), dy2 = p2.y() - p1.y();
  double dx3 = p3.x() - p1.x(), dy3 = p3.y() - p1.y();
  double d = dx2 * dy3 - dy2 * dx3;

  if (std::abs(d) < 1e-10) return 0.0;

  double x13 = p1.x() * p1.x() + p1.y() * p1.y();
  double x23 = p2.x() * p2.x() + p2.y() * p2.y();
  double x33 = p3.x() * p3.x() + p3.y() * p3.y();
  double cx = 0.5 * ((x13 - x23) * dy3 - (x13 - x33) * dy2) / d;
  double cy = 0.5 * (dx2 * (x13 - x33) - dx3 * (x13 - x23)) / d;

  if (center) { center->setX(cx); center->setY(cy); }
  return 1.0 / std::hypot(cx - p1.x(), cy - p1.y());
}

}  // namespace abot_rpp
#endif
