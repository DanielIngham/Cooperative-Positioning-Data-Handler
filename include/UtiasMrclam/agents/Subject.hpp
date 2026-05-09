/**
 * @file Subject.hpp
 */
#pragma once

#include <cstddef>
#include <functional>

namespace utias::mrclam::agent {

/**
 * Abstract class for housing a readonly subject number.
 */
class Subject {
public:
  Subject() = default;
  Subject(Subject &&) = default;
  Subject(const Subject &) = default;
  Subject &operator=(Subject &&) = default;
  Subject &operator=(const Subject &) = default;
  ~Subject() = default;

  Subject(unsigned short subject) : value_{subject} {};

  unsigned short value() const { return value_; }

  bool operator==(const Subject &other) const {
    return value_ == other.value();
  }

  bool operator<(const Subject &other) const { return value_ < other.value(); }

private:
  unsigned short value_{};
};
} // namespace utias::mrclam::agent

namespace std {

template <> struct hash<utias::mrclam::agent::Subject> {
  size_t operator()(const utias::mrclam::agent::Subject &sub) const {
    return hash<unsigned short>()(sub.value());
  }
};

} // namespace std
