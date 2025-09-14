
#include "Landmark.h"
#include "Agent.h"
#include <stdexcept>
#include <string>

namespace Data {

Landmark::Landmark(unsigned short id, unsigned short barcode)
    : Agent(id, barcode, Agent::Type::LANDMARK) {}

const double Landmark::x() const { return x_; }
const double Landmark::y() const { return y_; }
const double Landmark::x_std_dev() const { return x_std_dev_; }
const double Landmark::y_std_dev() const { return y_std_dev_; }

void Landmark::position(double x, double y) {
  static bool set{};
  if (set) {
    throw std::runtime_error("Landmark " + id() +
                             " position can only be set once.");
  }

  x_ = x;
  y_ = y;

  set = true;
}

void Landmark::standard_deviation(double x, double y) {
  static bool set{};
  if (set) {
    throw std::runtime_error("Landmark " + id() +
                             " standard deviation can only be set once.");
  }

  x_std_dev_ = x;
  y_std_dev_ = y;

  set = true;
}

} // namespace Data
