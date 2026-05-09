#include "UtiasMrclam/extractors/GroundtruthExtractor.hpp"
#include "UtiasMrclam/extractors/RobotExtractor.hpp"

#include <cassert>
#include <sstream>
#include <string>

namespace utias::mrclam {

const std::vector<geo::PoseStamped>
GroundtruthExtractor::readData(const std::string &dataset_directory,
                               const agent::Subject &agent_subject_number) {

  const std::string &file_directory{
      RobotExtractor::getFileDirectory(dataset_directory, agent_subject_number,
                                       RobotExtractor::DataType::GROUNDTRUTH)};

  return readInternal<std::vector<geo::PoseStamped>>(
      file_directory, [](const std::string &line, auto &groundtruth_data) {
        double time, x, y, orientation;

        std::istringstream iss(line);
        iss >> time >> x >> y >> orientation;

        groundtruth_data.emplace_back(time, x, y, orientation);
      });
}

} // namespace utias::mrclam
