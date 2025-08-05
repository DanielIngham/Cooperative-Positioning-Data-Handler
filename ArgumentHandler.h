/**
 * @file ArgumentHandler.h
 * @brief Header file containing functionality for parsing arguments given to
 * the executable into the Data::Handler class.
 * @author Daniel Ingham
 * @date 2025-08-01
 */
#ifndef INCLUDE_DATAHANDLER_INPUTHANDLER_HPP_
#define INCLUDE_DATAHANDLER_INPUTHANDLER_HPP_

#include <iostream>
#include <stdexcept>
#include <string>

#include "DataHandler.h"
#include "Simulator.h"

namespace ArgumentHandler {

enum MyEnum {
  EXECUTABLE, ///< The first argument is always the executable.
  FLAG,       ///< The second argument should be a flag prefaced with '-'.
  ARG         ///< The third argument should be the value
};

/**
 * Returns a string telling the user how to access the help menu.
 * @param executable name of the executable file.
 */
inline std::string helpMessage(char *executable) {
  return "Help Menu: " + std::string(executable) + " -h";
}

/**
 * Checks whether the argument provided is a flag (i.e. has a '-' prefix)
 * @param flag argument provided by the user.
 * @returns Flag indicating whether the argument is a flag.
 */
inline bool checkFlag(char *flag) {
  if (flag[0] != '-') {
    std::cout << "First argument must be a flag." << std::endl;
    return false;
  }
  return true;
}

/**
 * Help menu explaining to the user how to pass arguments to the executable.
 * @param executable name of the executable.
 */
inline void showHelp(char *executable) {
  std::cout << "\033[1;32m"
            << "COOPERATIVE LOCALISATION USING FACTOR GRAPH OPTIMISATION\033[0m"
            << std::endl;

  std::cout << std::endl
            << "Usage: " << executable << " <flag> <args>\n"
            << std::endl;

  std::cout << "<flag>" << std::endl
            << '\t' << "-h\t " << "Help Menu" << std::endl
            << '\t' << "-d\t " << "Dataset selector" << std::endl
            << '\t' << "-s\t " << "Simulator selector" << std::endl
            << std::endl;

  std::cout << "Required <args>" << std::endl
            << '\t' << "-d <1:9>\t " << "Dataset number [1 through 9]"
            << std::endl
            << '\t' << "-s <data points>\t " << "Number of simulated datapoints"
            << std::endl
            << std::endl;

  std::cout << "Optional Simulation flags" << std::endl
            << '\t' << "-o <output directory>" << std::endl
            << '\t' << "-s <sample period>" << std::endl
            << std::endl;

  std::cout << "Optional Simulation flags" << std::endl
            << '\t' << "-o <output directory>" << std::endl
            << '\t' << "-p <sample period>" << std::endl
            << '\t' << "-r <number of robots>" << std::endl
            << '\t' << "-r <number of robots>" << std::endl
            << '\t' << "-l <number of landmarks>" << std::endl
            << '\t' << "-S <Simulation seed>" << std::endl
            << std::endl;
}

/**
 * Takes an argument and converts it to an integer.
 * @param argument the argument that needs to be converted into an integer.
 */
inline int argToInt(char *argument) {
  int argument_integer;

  try {
    argument_integer = std::stoi(argument);

  } catch (const std::invalid_argument &e) {
    std::cerr << "Invalid argument: not a number" << std::endl;
    throw;

  } catch (const std::out_of_range &e) {
    std::cerr << "Number out of range" << std::endl;
    throw;
  }

  /* There are only 9 UTIAS datasets. */
  if (argument_integer < 1 || argument_integer > 9) {
    throw std::runtime_error("Select a dataset between 1 and 9.");
  }

  return argument_integer;
}

/**
 * Takes an argument and converts it to an double.
 * @param argument the argument that needs to be converted into an integer.
 */
inline double argToDouble(char *argument) {
  double argument_float;

  try {
    argument_float = std::stod(std::string(argument));

  } catch (const std::invalid_argument &e) {
    std::cerr << "Invalid argument: not a number" << std::endl;
    throw;

  } catch (const std::out_of_range &e) {
    std::cerr << "Number out of range" << std::endl;
    throw;
  }

  return argument_float;
}

/**
 * Given the user has provided arguments pertaining to reading data from a
 * given data set, this function is responsible for setting the dataset options.
 * @param argc Argument counter. Number of arguments provided by the user.
 * @param argv Argument values. The strings corresponding to the arguments.
 * @param data Data::Handler instance to be set based on provided arguments.
 */
inline void setDataSet(int argc, char *argv[], Data::Handler &data) {

  std::string dataset;
  std::string output_directory = Data::HandlerDefaults::kOutputDir;
  double sample_period = Data::HandlerDefaults::kSamplePeriod;

  if (argc == 2) {
    throw std::runtime_error("No dataset provided. " +
                             helpMessage(argv[EXECUTABLE]));
  }

  int dataset_number = argToInt(argv[ARG]);

  /* There are only 9 UTIAS datasets. */
  if (dataset_number < 1 || dataset_number > 9) {
    throw std::runtime_error("Select a dataset between 1 and 9.");
  }

  dataset = "MRCLAM_Dataset" + std::string(argv[ARG]);

  for (int i = 3; i < argc; i += 2) {

    if (!checkFlag(argv[i])) {
      return;
    }

    switch (argv[i][1]) {

      /* Output Directory */
    case 'o':
      output_directory = std::string(argv[i + 1]);
      break;

      /* Sample Period */
    case 'p':
      sample_period = argToDouble(argv[i + 1]);
      break;

    default:
      throw std::runtime_error("Unknown option: " + std::string(argv[i]));
      break;
    }
  }

  /* NOTE: If the output directory is not specified by the user, the set the
   * output directory to the executables name. */
  if (output_directory == Data::HandlerDefaults::kOutputDir) {
    output_directory = std::string(argv[EXECUTABLE]);
  }

  std::cout << "\033[1;32m" << "Dataset Options" << "\033[0m" << std::endl
            << '\t' << "- Dataset: " << dataset << std::endl
            << '\t' << "- Sample Period: " << sample_period << std::endl
            << '\t' << "- Output Directory: " << output_directory << std::endl
            << std::endl;

  data.setDataSet(dataset, output_directory, sample_period);
}

/**
 * Given the user has provided arguments pertaining to simulating cooperative
 * localisation data, this function is responsible for setting the simulation
 * options.
 * @param argc Argument counter. Number of arguments provided by the user.
 * @param argv Argument values. The strings corresponding to the arguments.
 * @param data Data::Handler instance to be set based on provided arguments.
 */
inline void setSimulation(int argc, char *argv[], Data::Handler &data) {
  int data_points = 7500U;
  int robots = Data::SimulationDefaults::kRobots;
  int landmarks = Data::SimulationDefaults::kLandmarks;
  int seed = Data::SimulationDefaults::kSeed;

  double sample_period = Data::SimulationDefaults::kSamplePeriod;

  std::string output_directory = Data::SimulationDefaults::kOutputDir;

  if (argc == 2) {
    throw std::runtime_error("Number of datapoints not specified. " +
                             helpMessage(argv[EXECUTABLE]));
  }

  for (int i = 3; i < argc; i += 2) {

    if (!checkFlag(argv[i])) {
      return;
    }

    switch (argv[i][1]) {

      /* Output Directory */
    case 'o':
      output_directory = std::string(argv[i + 1]);
      break;

      /* Sample Period */
    case 'p':
      sample_period = argToDouble(argv[i + 1]);
      break;

      /* Number of Robots */
    case 'r':
      robots = argToInt(argv[i + 1]);
      break;

      /* Number of Landmarks */
    case 'l':
      landmarks = argToInt(argv[i + 1]);
      break;

    /* Seed */
    case 'S':
      argToInt(argv[i + 1]);
      break;

    default:
      throw std::runtime_error("Unknown option: " + std::string(argv[i]));
      break;
    }
  }

  /* NOTE: If the output directory is not specified by the user, the set the
   * output directory to the executables name. */
  if (output_directory == Data::SimulationDefaults::kOutputDir) {
    output_directory = std::string(argv[EXECUTABLE]);
  }

  std::cout << "\033[1;32m" << "Simulation Options" << "\033[0m" << std::endl
            << '\t' << "- Data points: " << data_points << std::endl
            << '\t' << "- Robots: " << robots << std::endl
            << '\t' << "- Landmarks: " << robots << std::endl
            << '\t' << "- Sample Period: " << sample_period << std::endl
            << '\t' << "- Output Directory: " << output_directory << std::endl
            << '\t' << "- Seed: " << seed << std::endl
            << std::endl;

  data.setSimulation(data_points, robots, landmarks, sample_period,
                     output_directory, seed);
}

/**
 * Checks the arguments provided by the user and sets the appropriate setting
 * for the data handler.
 * @param argc Argument counter. Number of arguments provided by the user.
 * @param argv Argument values. The strings corresponding to the arguments.
 * @param data Data::Handler instance to be set based on provided arguments.
 */
inline void setArguments(int argc, char *argv[], Data::Handler &data) {

  /* Check if an argument is provided. */
  if (argc == 1) {
    std::cout << "\033[1;31m" << "[ERROR] " << "\033[0m"
              << "No arguments provided." << std::endl
              << helpMessage(argv[EXECUTABLE]) << std::endl;
    return;
  }

  /* Check if the first argument is a flag. */
  if (!checkFlag(argv[FLAG])) {
    std::cout << helpMessage(argv[EXECUTABLE]) << std::endl;
    return;
  }

  switch (argv[FLAG][1]) {
    /* Help Menu */
  case 'h':
    showHelp(argv[EXECUTABLE]);
    break;

    /* Dataset Selector */
  case 'd':
    setDataSet(argc, argv, data);
    break;

    /* Simulator Selector */
  case 's':
    setSimulation(argc, argv, data);
    break;

  default:
    throw std::runtime_error("Unknown option: " + std::string(argv[FLAG]));
  }
}

} // namespace ArgumentHandler
#endif // INCLUDE_DATAHANDLER_INPUTHANDLER_HPP_
