#pragma once

namespace utias::mrclam::geo {
class Point {
public:
  Point() = default;
  Point(Point &&) = default;
  Point(const Point &) = default;
  Point &operator=(Point &&) = default;
  Point &operator=(const Point &) = default;
  ~Point() = default;

  Point(double x, double y) : x_{x}, y_{y} {};

  double x() const { return x_; }
  double y() const { return y_; }

  Point operator+(const Point &point2) {
    double x{this->x() + point2.x()};
    double y{this->y() + point2.y()};

    return {x, y};
  }

  Point operator-(const Point &point2) {
    double x{this->x() - point2.x()};
    double y{this->y() - point2.y()};

    return {x, y};
  }

  Point operator*(const Point &point2) {
    double x{this->x() * point2.x()};
    double y{this->y() * point2.y()};

    return {x, y};
  }

private:
  double x_;
  double y_;
};

} // namespace utias::mrclam::geo
