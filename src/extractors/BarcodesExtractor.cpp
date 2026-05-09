#include "UtiasMrclam/extractors/BarcodesExtractor.hpp"

#include <sstream>

namespace utias::mrclam {

agent::IdentifierMap
BarcodesExtractor::readData(const std::string &dataset_directory) {

  std::string file{dataset_directory + "/Barcodes.dat"};

  return readInternal<agent::IdentifierMap>(

      file, [](const std::string &line, agent::IdentifierMap &map) {
        std::istringstream iss(line);
        unsigned short raw_subject;
        size_t raw_barcode;

        if (iss >> raw_subject >> raw_barcode) {
          agent::Subject subject{raw_subject};
          agent::Barcode barcode{raw_barcode};

          map.insert(subject, barcode);
        }
      });
}

} // namespace utias::mrclam
