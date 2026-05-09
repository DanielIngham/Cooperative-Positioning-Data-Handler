#include "UtiasMrclam/agents/Landmark.hpp"
#include "UtiasMrclam/agents/Agent.hpp"

namespace Data {

Landmark::Landmark(unsigned short id, unsigned short barcode)
    : Agent(id, barcode, Agent::Type::LANDMARK) {}

const double Landmark::x() const { return position_.x; }
const double Landmark::y() const { return position_.y; }
const double Landmark::x_std_dev() const { return std_dev_.x; }
const double Landmark::y_std_dev() const { return std_dev_.y; }

void Landmark::position(double x, double y) {

  position_.x = x;
  position_.y = y;
}

Landmark::~Landmark() {}

void Landmark::standard_deviation(double x, double y) {

  std_dev_.x = x;
  std_dev_.y = y;
}

} // namespace Data
