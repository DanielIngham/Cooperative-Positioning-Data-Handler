/**
 * @file Barcode.hpp
 */
#pragma once

#include <cstddef>
#include <functional>

namespace utias::mrclam::agent {
/**
 * Abstract class for housing a readonly barcode.
 */
class Barcode {
public:
  Barcode() = default;
  Barcode(Barcode &&) = default;
  Barcode(const Barcode &) = default;
  Barcode &operator=(Barcode &&) = default;
  Barcode &operator=(const Barcode &) = default;
  ~Barcode() = default;

  Barcode(size_t barcode) : value_{barcode} {};

  double value() const { return value_; }

  bool operator==(const Barcode &other) const {
    return value_ == other.value();
  }

  bool operator<(const Barcode &other) const { return value_ < other.value(); }

private:
  size_t value_{};
};
} // namespace utias::mrclam::agent

namespace std {

template <> struct hash<utias::mrclam::agent::Barcode> {
  size_t operator()(const utias::mrclam::agent::Barcode &bar) const {
    return hash<unsigned short>()(bar.value());
  }
};

} // namespace std
