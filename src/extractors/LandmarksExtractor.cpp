#include "UtiasMrclam/extractors/LandmarksExtractor.hpp"
#include "UtiasMrclam/agents/IdentifierMap.hpp"
#include "UtiasMrclam/agents/Subject.hpp"

#include <cmath>
#include <sstream>

namespace utias::mrclam {

std::map<agent::Subject, geo::PointWithCovariance>
LandmarksExtractor::readData(const std::string &dataset_directory,
                             agent::IdentifierMap identifier_map) {

  std::string file{dataset_directory + "/Landmark_Groundtruth.dat"};

  return readInternal<std::map<agent::Subject, geo::PointWithCovariance>>(
      file, [&identifier_map](
                const std::string &line,
                std::map<agent::Subject, geo::PointWithCovariance> &map) {
        unsigned short raw_subject_number;
        double x, y, x_std_dev, y_std_dev;
        std::istringstream iss(line);

        if (iss >> raw_subject_number >> x >> y >> x_std_dev >> y_std_dev) {

          agent::Subject subject{raw_subject_number};

          /* Convert standard deviation to variance. */
          double var_x{std::pow(x_std_dev, 2)}, var_y{std::pow(y_std_dev, 2)};

          map[subject] = geo::PointWithCovariance{x, y, var_x, var_y};
        }
      });
}

} // namespace utias::mrclam
