#pragma once

#include "UtiasMrclam/agents/Identifier.hpp"
#include "UtiasMrclam/extractors/FileExtractor.hpp"
#include "UtiasMrclam/sensors/MeasurementStamped.hpp"

#include <vector>

namespace utias::mrclam {

class MeasurementExtractor : FileExtractor {
public:
  MeasurementExtractor() = delete;

  [[nodiscard]] static const std::vector<sensor::MeasurementStamped>
  readData(const std::string &dataset_directory,
           const agent::Identifier &agent_ID,
           const agent::Identifier::bimap_t &identifier_map);

private:
};

} // namespace utias::mrclam
