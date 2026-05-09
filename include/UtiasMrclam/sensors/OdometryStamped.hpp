#pragma once
#include "UtiasMrclam/sensors/Odometry.hpp"

namespace utias::mrclam::sensor {
class OdometryStamped {
public:
  OdometryStamped() = delete;
  OdometryStamped(OdometryStamped &&) = default;
  OdometryStamped(const OdometryStamped &) = default;
  OdometryStamped &operator=(OdometryStamped &&) = default;
  OdometryStamped &operator=(const OdometryStamped &) = default;
  ~OdometryStamped() = default;

  OdometryStamped(double time_stamp, double forward_velocity,
                  double angular_velocity)
      : time_stamp_{time_stamp},
        odometry_{forward_velocity, angular_velocity} {};

private:
  double time_stamp_;
  Odometry odometry_;
};
} // namespace utias::mrclam::sensor
