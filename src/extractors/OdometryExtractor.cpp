#include "UtiasMrclam/extractors/OdometryExtractor.hpp"
#include "UtiasMrclam/extractors/RobotExtractor.hpp"

#include <sstream>

namespace utias::mrclam {

const std::vector<sensor::OdometryStamped>
OdometryExtractor::readData(const std::string &dataset_directory,
                            const agent::Subject &agent_subject_number) {

  const std::string &file_directory{RobotExtractor::getFileDirectory(
      dataset_directory, agent_subject_number.value(),
      RobotExtractor::DataType::ODOMETRY)};

  return readInternal<std::vector<sensor::OdometryStamped>>(
      file_directory, [](const std::string &line, auto &groundtruth_data) {
        double time, forward_velocity, angular_velocity;

        std::istringstream iss(line);
        iss >> time >> forward_velocity >> angular_velocity;

        groundtruth_data.emplace_back(time, forward_velocity, angular_velocity);
      });
}

} // namespace utias::mrclam
