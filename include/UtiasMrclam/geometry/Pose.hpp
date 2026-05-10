/**
 * @file Pose.hpp
 */
#include "UtiasMrclam/geometry/Point.hpp"
#include "UtiasMrclam/utils/Utils.hpp"

namespace utias::mrclam::geo {
class Pose {
public:
  Pose() = delete;
  Pose(Pose &&) = default;
  Pose(const Pose &) = default;
  Pose &operator=(Pose &&) = default;
  Pose &operator=(const Pose &) = default;
  ~Pose() = default;

  /**
   * Constructor that allows for the use to specify each attribute of the Pose
   * class individually.
   * @param x x component of the vehicle's coordinate.
   * @param y y component of the vehicle's coordinate.
   * @param orientation heading of the vehicle.
   */
  Pose(double x, double y, double orienation)
      : coordinate_{x, y}, orientation_{orienation} {}

  /**
   * Constructor that allows for the passing of a point object for the
   * coordinate of the vehicle.
   * @param coordinate x-y coordinate pair of the vehicle.
   * @param orientation heading of the vehicle.
   */
  Pose(Point coordinate, double orienation)
      : coordinate_{coordinate}, orientation_{orienation} {}

  /**
   * Getter for the x-y coordinate pair of the vehicle pose.
   */
  const Point &coordinate() const { return coordinate_; }
  /**
   * Getter for the x coordinate of the vehicle pose.
   */
  double x() const { return coordinate_.x(); }
  /**
   * Getter for the y coordinate of the vehicle pose.
   */
  double y() const { return coordinate_.y(); }
  /**
   * Getter for the vehicle's orientation.
   */
  double theta() const { return orientation_; }

  /**
   * Operator overload which allows for summing two poses. It is simply a
   * component wise summation (i.e. vehicle1.pose.x + vehicle1.pose.x, etc).
   * @note the operator overload also normalises the angle after performing the
   * operation.
   */
  Pose operator+(const Pose &pose2) {
    Point coordinate{this->coordinate_ + pose2.coordinate()};
    double theta{this->theta() + pose2.theta()};
    utils::normaliseAngle(theta);

    return {coordinate, theta};
  }
  /**
   * Operator overload which allows for subtraction two poses. It is simply a
   * component wise subraction (i.e. vehicle1.pose.x + vehicle1.pose.x, etc).
   * @note the operator overload also normalises the angle after performing the
   * operation.
   */
  Pose operator-(const Pose &pose2) {
    Point coordinate{this->coordinate_ - pose2.coordinate()};
    double theta{this->orientation_ - pose2.theta()};
    utils::normaliseAngle(theta);

    return {coordinate, theta};
  }
  /**
   * Operator overload which allows for subtraction two poses. It is simply a
   * component wise multiplication (i.e. vehicle1.pose.x + vehicle1.pose.x,
   * etc).
   * @note the operator overload also normalises the angle after performing the
   * operation.
   */
  Pose operator*(const Pose &pose2) {
    Point coordinate{this->coordinate_ * pose2.coordinate()};
    double theta{this->theta() * pose2.theta()};
    utils::normaliseAngle(theta);

    return {coordinate, theta};
  }

private:
  /**
   * x-y coordinate pair of the vehicles pose.
   */
  Point coordinate_;
  /**
   * Heading of vehicle.
   */
  double orientation_;
};

} // namespace utias::mrclam::geo
