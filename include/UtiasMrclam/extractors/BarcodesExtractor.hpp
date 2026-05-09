/**
 * @file BarcodesExtractor.hpp
 */

#include "UtiasMrclam/agents/Barcode.hpp"
#include "UtiasMrclam/agents/IdentifierMap.hpp"
#include "UtiasMrclam/agents/Subject.hpp"
#include "UtiasMrclam/extractors/FileExtractor.hpp"
#include <boost/bimap.hpp>

#include <string>

namespace utias::mrclam {

class BarcodesExtractor : private FileExtractor {
public:
  BarcodesExtractor() = delete;

  [[nodiscard]] static agent::IdentifierMap
  readData(const std::string &dataset_directory);
};
} // namespace utias::mrclam
