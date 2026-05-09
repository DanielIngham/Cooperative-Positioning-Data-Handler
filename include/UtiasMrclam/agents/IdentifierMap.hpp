#pragma once

#include "UtiasMrclam/agents/Barcode.hpp"
#include "UtiasMrclam/agents/Subject.hpp"

#include <unordered_map>

namespace utias::mrclam::agent {

class IdentifierMap {
public:
  IdentifierMap() = default;
  IdentifierMap(IdentifierMap &&) = default;
  IdentifierMap(const IdentifierMap &) = default;
  IdentifierMap &operator=(IdentifierMap &&) = default;
  IdentifierMap &operator=(const IdentifierMap &) = default;
  ~IdentifierMap() = default;

  void insert(const Subject &subject, const Barcode &barcode);
  Barcode at(const Subject &subject) const;
  Subject at(const Barcode &barcode) const;

private:
  std::unordered_map<Subject, Barcode> barcodes_;
  std::unordered_map<Barcode, Subject> subjects_;
};
} // namespace utias::mrclam::agent
