/**
 * @file Common.hpp
 * Contains functionality that could be used by classes throughout the project.
 */
#include <algorithm>
#include <cmath>
#include <numbers>

#include "UtiasMrclam/agents/Robot.hpp"

namespace utias::mrclam::utils {
/**
 * Normalises a given angle to fall in the range [-pi, pi).
 * @param angle_rad unnormalised angle in radians
 * @returns an equivalent angle normalised to fall within the range [-pi,
 * pi).
 */
inline void normaliseAngle(double &angle_rad) {
  angle_rad -= 2.0 * std::numbers::pi *
               floor((angle_rad + std::numbers::pi) / (2.0 * std::numbers::pi));
}

inline const Robot::Measurement *
getMeasurement(const std::vector<Robot::Measurement> &measurements,
               double time) {

  /* Threshold for double floating point precision. */
  static constexpr double decimal_threshold{1e-5};

  /* Find the first element whose time is larger than the current time.  */
  auto iterator{
      std::lower_bound(measurements.begin(), measurements.end(), time,
                       [](const Robot::Measurement &measurement, double time) {
                         return measurement.time < time;
                       })};

  if (iterator == measurements.end())
    return nullptr;

  /* Check if the found element's time falls with the threshold of the current
   * time. Then check if the previous element's time falls within the
   * threshold of the current time. */
  const unsigned short check_current_and_previous_index{2U};
  for (unsigned short i{}; i < check_current_and_previous_index; ++i) {
    if (std::abs(iterator->time - time) < decimal_threshold) {

      return &(*iterator);
    }

    if (iterator == measurements.begin()) {
      break;
    }

    --iterator;
  }

  return nullptr;
}

/**
 * Return the measurement that has the same timestamp as the odometry input at
 * the element index provided.
 * @param robot_id ID of the robot whose measurement needs to be extracted.
 * @param index Synced index.
 * @returns pointer to the measurement found. std::nullptr if nothing was
 * found.
 */
inline const Robot::Measurement *getMeasurement(const Robot *robot,
                                                size_t index) {

  const Robot::Odometry &odometry{robot->synced.odometry.at(index)};
  const double current_time{odometry.time};

  const std::vector<Robot::Measurement> &measurements{
      robot->synced.measurements};

  return getMeasurement(measurements, current_time);
}

} // namespace utias::mrclam::utils
