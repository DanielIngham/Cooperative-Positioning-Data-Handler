#pragma once

#include <cstddef>

namespace utias::mrclam {

class DataSetExtractor {
public:
  DataSetExtractor() = delete;
  DataSetExtractor(DataSetExtractor &&) = default;
  DataSetExtractor(const DataSetExtractor &) = default;
  DataSetExtractor &operator=(DataSetExtractor &&) = default;
  DataSetExtractor &operator=(const DataSetExtractor &) = default;
  ~DataSetExtractor() = default;

  /**
   * Extracts the dataset specified by the index provided.
   * @param dataset_index The index of the dataset to be extracted. Could be
   * value in the range [1,9].
   */
  static void extractDataSet(const size_t dataset_index);

private:
  static constexpr const char *kdataset_names[]{
      "MRCLAM_Dataset1", "MRCLAM_Dataset2", "MRCLAM_Dataset3",
      "MRCLAM_Dataset4", "MRCLAM_Dataset5", "MRCLAM_Dataset6",
      "MRCLAM_Dataset7", "MRCLAM_Dataset8", "MRCLAM_Dataset9"};
};
} // namespace utias::mrclam
