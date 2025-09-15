
#include "Landmark.h"
#include "Agent.h"
#include <stdexcept>
#include <string>

namespace Data {

Landmark::Landmark(unsigned short id, unsigned short barcode)
    : Agent(id, barcode, Agent::Type::LANDMARK) {}

const double Landmark::x() const { return position_.x; }
const double Landmark::y() const { return position_.y; }
const double Landmark::x_std_dev() const { return std_dev_.x; }
const double Landmark::y_std_dev() const { return std_dev_.y; }

void Landmark::position(double x, double y) {
  if (position_.set) {
    throw std::runtime_error("Landmark " + id() +
                             " position can only be set once.");
  }

  position_.x = x;
  position_.y = y;

  position_.set = true;
}

Landmark::~Landmark() {}

void Landmark::standard_deviation(double x, double y) {
  if (std_dev_.set) {
    throw std::runtime_error("Landmark " + id() +
                             " standard deviation can only be set once.");
  }

  std_dev_.x = x;
  std_dev_.y = y;

  std_dev_.set = true;
}

} // namespace Data
