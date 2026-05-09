/**
 * @file DataSet.cpp
 */
#include "UtiasMrclam/DataSet.hpp"
#include <cassert>

namespace utias::mrclam {

DataSet::DataSet(size_t dataset_index) {

  assert(dataset_index >= 1 && dataset_index <= 9);

  dataset_index_ = dataset_index;
}

} // namespace utias::mrclam
