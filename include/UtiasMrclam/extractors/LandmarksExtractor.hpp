#pragma once

#include "UtiasMrclam/agents/Barcode.hpp"
#include "UtiasMrclam/agents/IdentifierMap.hpp"
#include "UtiasMrclam/agents/Subject.hpp"
#include "UtiasMrclam/extractors/FileExtractor.hpp"
#include "UtiasMrclam/geometry/PointWithCovariance.hpp"

#include <boost/bimap.hpp>
#include <map>

namespace utias::mrclam {

class LandmarksExtractor : FileExtractor {
public:
  LandmarksExtractor() = delete;

  [[nodiscard]] static std::map<agent::Subject, geo::PointWithCovariance>
  readData(const std::string &dataset_directory,
           agent::IdentifierMap identifier_map);

private:
};

} // namespace utias::mrclam
