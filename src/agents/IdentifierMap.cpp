#include "UtiasMrclam/agents/IdentifierMap.hpp"
#include "UtiasMrclam/agents/Barcode.hpp"
#include "UtiasMrclam/agents/Subject.hpp"
#include <stdexcept>
#include <string>

namespace utias::mrclam::agent {

void IdentifierMap::insert(const Subject &subject, const Barcode &barcode) {
  auto subject_it{barcodes_.find(subject)};
  auto barcode_it{subjects_.find(barcode)};

  if (subject_it != barcodes_.end())
    throw std::invalid_argument("Subject " + std::to_string(subject.value()) +
                                " already present in map.");

  if (barcode_it != subjects_.end())
    throw std::invalid_argument("Barcode " + std::to_string(barcode.value()) +
                                " already present in map.");

  barcodes_[subject] = barcode;
  subjects_[barcode] = subject;
}

Barcode IdentifierMap::at(const Subject &subject) const {
  return barcodes_.at(subject);
}

Subject IdentifierMap::at(const Barcode &barcode) const {
  return subjects_.at(barcode);
}

} // namespace utias::mrclam::agent
