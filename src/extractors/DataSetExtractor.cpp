#include "UtiasMrclam/extractors/DataSetExtractor.hpp"
#include "UtiasMrclam/agents/Subject.hpp"
#include "UtiasMrclam/extractors/BarcodesExtractor.hpp"
#include "UtiasMrclam/extractors/LandmarksExtractor.hpp"
#include "UtiasMrclam/extractors/RobotExtractor.hpp"

#include <cassert>

namespace utias::mrclam {

void DataSetExtractor::extractDataSet(const size_t dataset_index) {
  assert(dataset_index >= 1 && dataset_index <= 9);

  agent::IdentifierMap identifier_map{
      BarcodesExtractor::readData(kdataset_names[dataset_index])};

  std::map<agent::Subject, geo::PointWithCovariance> landmarks{
      LandmarksExtractor::readData(kdataset_names[dataset_index],
                                   identifier_map)};
  // TODO: Create a fleet extractor which loops through all robots and calls
  // robot extractor on each and then forms a map std::map<agent::Subject, ... >
  // where ... is some data structure which contains all the raw data. This will
  // form the first component in the builder, which will be passed on to the
  // synchroniser, then Groundtruth, then error.
  //
  // auto robots{FleetExtractor{}};
}

} // namespace utias::mrclam
