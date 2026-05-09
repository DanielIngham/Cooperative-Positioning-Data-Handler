#include "UtiasMrclam/extractors/RobotExtractor.hpp"
#include <sstream>

namespace utias::mrclam {

const std::string
RobotExtractor::getFileDirectory(const std::string &dataset_directory,
                                 const agent::Subject &agent_subject_number,
                                 const DataType &data_type) {
  std::ostringstream ss;
  const std::string &subject{std::to_string(agent_subject_number.value())};
  ss << dataset_directory << "/Robot" << subject << "_";

  switch (data_type) {
  case DataType::GROUNDTRUTH:
    ss << "Groundtruth.dat";
    break;
  case DataType::ODOMETRY:
    ss << "Odometry.dat";
    break;
  case DataType::MEASUREMENT:
    ss << "Measurement.dat";
    break;
  }

  return ss.str();
}
} // namespace utias::mrclam
