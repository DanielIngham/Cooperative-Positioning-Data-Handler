/**
 * @file GroundtruthExtractor.hpp
 */
#pragma once

#include "UtiasMrclam/agents/Subject.hpp"
#include "UtiasMrclam/extractors/FileExtractor.hpp"
#include "UtiasMrclam/geometry/PoseStamped.hpp"

#include <string>
#include <vector>

namespace utias::mrclam {
/**
 * Extracts the groundtruth data for a given robot from the
 * Robotx_Groundtruth.dat file.
 * @note It's worth clarifying the the extractor only extracts data for one
 * agent.
 */
class GroundtruthExtractor : FileExtractor {
public:
  /**
   * Prevents creation of an instance for a class that simply houses static
   * functions.
   */
  GroundtruthExtractor() = delete;

  /**
   * Extracts the groundruth data for a given agent from Robotx_Groundtruth.dat.
   * @param dataset_directory The parent directory of the groundtruth data.
   * @param agent_ID Idenifier for the given agent.
   */
  [[nodiscard]] static const std::vector<geo::PoseStamped>
  readData(const std::string &dataset_directory,
           const agent::Subject &agent_subject_number);

private:
};
} // namespace utias::mrclam
