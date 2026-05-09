/**
 * @file Common.hpp
 * Contains functionality that could be used by classes throughout the project.
 */
#include <cmath>
#include <numbers>
#include <string>

namespace utias::mrclam::common {

/**
 * Normalises a given angle to fall in the range [-pi, pi).
 * @param angle_rad unnormalised angle in radians
 * @returns an equivalent angle normalised to fall within the range [-pi,
 * pi).
 */
inline double normaliseAngle(double angle_rad) {
  angle_rad = std::fmod(angle_rad + std::numbers::pi, 2.0 * std::numbers::pi);

  if (angle_rad < 0)
    angle_rad += 2.0 * std::numbers::pi;

  return angle_rad - std::numbers::pi;
}
} // namespace utias::mrclam::common
