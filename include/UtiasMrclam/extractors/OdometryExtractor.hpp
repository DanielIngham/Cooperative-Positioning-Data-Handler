#pragma once

#include "UtiasMrclam/agents/Subject.hpp"
#include "UtiasMrclam/extractors/FileExtractor.hpp"
#include "UtiasMrclam/sensors/OdometryStamped.hpp"

#include <vector>

namespace utias::mrclam {
class OdometryExtractor : FileExtractor {
public:
  OdometryExtractor() = delete;

  [[nodiscard]] static const std::vector<sensor::OdometryStamped>
  readData(const std::string &dataset_directory,
           const agent::Subject &agent_subject_number);

private:
};
} // namespace utias::mrclam
