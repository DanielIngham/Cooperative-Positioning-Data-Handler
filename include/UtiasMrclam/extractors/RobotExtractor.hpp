/**
 * @file RobotExtractor.hpp
 */
#pragma once

#include "UtiasMrclam/agents/Subject.hpp"

#include <string>

namespace utias::mrclam {

class RobotExtractor {
public:
  RobotExtractor() = default;
  RobotExtractor(RobotExtractor &&) = default;
  RobotExtractor(const RobotExtractor &) = default;
  RobotExtractor &operator=(RobotExtractor &&) = default;
  RobotExtractor &operator=(const RobotExtractor &) = default;
  ~RobotExtractor() = default;

  enum class DataType { GROUNDTRUTH, ODOMETRY, MEASUREMENT };

  static const std::string
  getFileDirectory(const std::string &dataset_directory,
                   const agent::Subject &agent_subject_number,
                   const DataType &data_type);

private:
};

} // namespace utias::mrclam
