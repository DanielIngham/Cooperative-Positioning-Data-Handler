#pragma once

namespace utias::mrclam::sensor {
class MeasurementStamped {
public:
  MeasurementStamped() = default;
  MeasurementStamped(MeasurementStamped &&) = default;
  MeasurementStamped(const MeasurementStamped &) = default;
  MeasurementStamped &operator=(MeasurementStamped &&) = default;
  MeasurementStamped &operator=(const MeasurementStamped &) = default;
  ~MeasurementStamped() = default;

private:
  /**
   * Time (in seconds).
   */
  double time_stamp_;
  /**
   * Measured range (in meters) from another agent.
   */
  double range_;
  /**
   * Measured bearing (in radians) counter-clock from east.
   */
  double bearing_;
};
} // namespace utias::mrclam::sensor
