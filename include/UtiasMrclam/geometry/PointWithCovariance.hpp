#include "UtiasMrclam/geometry/Point.hpp"
#include <array>

namespace utias::mrclam::geo {

class PointWithCovariance {
public:
  PointWithCovariance() = default;
  PointWithCovariance(PointWithCovariance &&) = default;
  PointWithCovariance(const PointWithCovariance &) = default;
  PointWithCovariance &operator=(PointWithCovariance &&) = default;
  PointWithCovariance &operator=(const PointWithCovariance &) = default;
  ~PointWithCovariance() = default;

  PointWithCovariance(double x, double y, std::array<double, 4> covariance)
      : point_{x, y}, covariance_{covariance} {}

  /**
   * Constructor when the noise associated with the x and y axis of the point is
   * uncorrelated.
   * @param x x-coordinate
   * @param y y-coordinate
   * @param var-x x-coordinate noise/error variance
   * @param var-y y-coordinate noise/error variance
   */
  PointWithCovariance(double x, double y, double var_x, double var_y)
      : point_{x, y}, covariance_{x, .0, .0, y} {}

private:
  Point point_;
  std::array<double, 4> covariance_;
};
} // namespace utias::mrclam::geo
