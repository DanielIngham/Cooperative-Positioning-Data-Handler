/**
 * @file DataSet.hpp
 */
#pragma once

#include <cstddef>
#include <string>

namespace utias::mrclam {

class DataSet {
public:
  DataSet() = delete;
  DataSet(DataSet &&) = default;
  DataSet(const DataSet &) = default;
  DataSet &operator=(DataSet &&) = default;
  DataSet &operator=(const DataSet &) = default;
  ~DataSet() = default;

  /**
   * Extracts the dataset specified by the index provided.
   * @param dataset_index The index of the dataset to be extracted. Could be
   * value in the range [1,9].
   */
  DataSet(size_t dataset_index);

private:
  /**
   * Index of the dataset to be extracted. Can be a value in the range [1,9].
   */
  size_t dataset_index_;
};
} // namespace utias::mrclam
