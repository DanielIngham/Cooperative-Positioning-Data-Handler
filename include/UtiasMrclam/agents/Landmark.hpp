/**
 * @file Landmark.h
 * @brief Header file of the Landmark struct.
 * @author Daniel Ingham
 * @date 2025-04-16
 */
#ifndef INCLUDE_INCLUDE_LANDMARK_H_
#define INCLUDE_INCLUDE_LANDMARK_H_

#include "UtiasMrclam/agents/Agent.hpp"
#include <vector>

namespace Data {

/**
 * @struct Landmark
 * @brief Houses all data related to a given landmark in the UTIAS multi-robot
 * localisation and mapping dataset.
 * @details This struct contains all the data found in the Landmarks.dat files.
 * The DataExtractor::readLandmarks is responsible for populating this data
 * structure with the appropriate values from a provided dataset.
 */
class Landmark : public Agent {
public:
  Landmark(unsigned short id, unsigned short barcode);
  Landmark() = default;
  Landmark(Landmark &&) = default;
  Landmark(const Landmark &) = default;
  Landmark &operator=(Landmark &&) = default;
  Landmark &operator=(const Landmark &) = default;
  ~Landmark();

  using List = std::vector<Landmark>;

  const double x() const;
  const double y() const;
  const double x_std_dev() const;
  const double y_std_dev() const;

  void position(double x, double y);
  void standard_deviation(double x, double y);

  friend std::ostream &operator<<(std::ostream &os, const Landmark &landmark) {
    return os << "[" << landmark.id() << " | " << landmark.barcode() << "]"
              << "\t x(" << landmark.x() << ")" << ", y(" << landmark.y()
              << ")";
  }

private:
  struct Euclidean2D {
    double x; ///< The landmark's global x-coordinate.
    double y; ///< The landmark's global y-coordinate.
  } position_, std_dev_;
};

} // namespace Data

#endif // INCLUDE_INCLUDE_LANDMARK_H_
