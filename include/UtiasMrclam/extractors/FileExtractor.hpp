/**
 * @file FileExtractor.hpp
 */
#pragma once

#include <fstream>
#include <string>

namespace utias::mrclam {

class FileExtractor {
protected:
  template <typename T, typename Parser>
  static T readInternal(const std::string &path, Parser parseLine) {

    std::ifstream file(path);
    if (!file.is_open())
      throw std::runtime_error("Could not open " + path);

    T container;
    std::string line;

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      parseLine(line, container);
    }
    return container;
  }
};

} // namespace utias::mrclam
