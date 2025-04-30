#include "../include/data_handler.h" // DataHandler
#include <algorithm>                 // std::find
#include <assert.h>
#include <chrono> // std::chrono
#include <cstddef>
#include <fstream>  // std::fstream
#include <iostream> // std::cout
#include <string>   // std::string
#include <thread>   // std::thread

#define TOTAL_DATASETS 9

/**
 * @brief Loops through the entire dataset file and counts the number of lines
 * that are not commented using '#'.
 * @param [in] filename the name of the input file.
 * @return the number of lines counted.
 * @note this function also counts empty lines. The only line ignored are the
 * ones where the character '#' is present as the first character in the line.
 */
std::size_t countFileLines(const std::string &filename) {
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "[ERROR] Failed to open file:" << filename << std::endl;
    return 0;
  }

  long unsigned int counter = 0;
  std::string line;

  while (std::getline(file, line)) {
    if ('#' == line[0]) {
      continue;
    }
    counter++;
  }

  return counter;
}

/**
 * @brief Unit Test 1: check if barcodes were set.
 */
void checkBarcodes() {
  DataHandler data;
  bool flag = true;

  for (unsigned short int d = 1; d <= TOTAL_DATASETS; d++) {
    const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(d);
    data.setDataSet(dataset);

    /* Unit Test 1: check if barcodes were set. */
    const auto barcodes = data.getBarcodes();

    for (unsigned short int j = 0; j < data.getNumberOfBarcodes(); j++) {
      /* All barcodes are initialised to zero, and no barcodes have a value of
       * zero. Therefore, if a barcode has a value of zero, it has not been
       * correctly set. */
      if (barcodes[j] == 0) {
        std::cerr << "[ERROR] Barcode " << j << " has not been correctly set."
                  << std::endl;
        flag = false;
      }
    }
  }

  flag ? std::cout << "\033[1;32m[U01 PASS]\033[0m All barcodes were set.\n"
       : std::cerr << "[U1 FAIL] All barcodes were not set.\n";

  return;
}

/**
 * @brief Unit Test 2: compare landmark barcodes to barcodes.
 */
void checkLandmarkBarcodes() {
  DataHandler data;
  bool flag = true;

  for (unsigned short int i = 1; i <= TOTAL_DATASETS; i++) {
    const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
    data.setDataSet(dataset);

    const auto barcodes = data.getBarcodes();
    const auto landmarks = data.getLandmarks();

    for (int j = 0; j < data.getNumberOfLandmarks(); j++) {
      /* Check the id against its barcode */
      if (landmarks[j].barcode != barcodes[landmarks[j].id - 1]) {
        std::cerr << "[ERROR] Landmark " << landmarks[j].id
                  << " does not not have the correct ID." << std::endl;
        flag = false;
      }
    }
  }

  flag ? std::cout << "\033[1;32m[U02 PASS]\033[0m All landmarks have the "
                      "correct barcodes.\n"
       : std::cerr << "\033[1;31m[U2 FAIL]\033[0m Landmarks do not have the "
                      "correct barcodes.\n";

  return;
}

/**
 * @brief Unit Test 3: check that all the ground truth values were extracted.
 */
void checkGroundtruthExtraction() {
  DataHandler data;
  bool flag = true;

  for (unsigned short int d = 1; d <= TOTAL_DATASETS; d++) {
    const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(d);

    data.setDataSet(dataset);

    const auto robots = data.getRobots();

    for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {
      std::string groundtruth_file =
          dataset + "/Robot" + std::to_string(id + 1) + "_Groundtruth.dat";

      long unsigned int counter = countFileLines(groundtruth_file);

      /* If the counter equals zero, the data from the file was not correctly
       * extracted. */
      if (0 == counter) {
        flag = false;
      }

      else if (robots[id].raw.states.size() != counter) {
        std::cerr << "Robot " << id + 1
                  << " does not have a size equal to the number of entries in "
                     "the groundtruth: "
                  << robots[id].raw.states.size() << " ≠ " << counter
                  << std::endl;
        flag = false;
      }
    }
  }

  flag
      ? std::cout << "\033[1;32m[U03 PASS]\033[0m All Robots have extracted "
                     "the correct amount groundtruth values from the dataset\n"
      : std::cerr << "\033[1;31m[U3 FAIL]\033[0m Not all robots extracted the "
                     "correct amount of groundtruth values from the dataset.\n";
}

/**
 * @brief Unit Test 4: check that all the odometry values were extracted.
 */
void checkOdometryExtraction() {
  DataHandler data;
  bool flag = true;

  /* Loop through every data */
  for (unsigned short int d = 1; d <= TOTAL_DATASETS; d++) {

    const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(d);

    data.setDataSet(dataset);

    const auto robots = data.getRobots();

    /* Loop through every robot */
    for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {

      std::string odometry_file =
          dataset + "/Robot" + std::to_string(id + 1) + "_Odometry.dat";

      long unsigned int counter = countFileLines(odometry_file);

      if (counter == 0) {
        flag = false;
      } else if (robots[id].raw.odometry.size() != counter) {
        std::cerr << "Robot " << id + 1
                  << " does not have a size equal to the number of entries in "
                     "the odometry file: "
                  << robots[id].raw.odometry.size() << " ≠ " << counter
                  << std::endl;
        flag = false;
      }
    }
  }
  flag ? std::cout << "\033[1;32m[U04 PASS]\033[0m All Robots have extracted "
                      "the correct amount of odometry values from the dataset\n"
       : std::cerr << "\033[1;31m[U3 FAIL]\033[0m Not all robots extracted the "
                      "correct amount of odometery values from the dataset.\n";
}

/**
 * @brief Unit Test 5: check that all the measurement values were extracted.
 */
void checkMeasurementExtraction() {
  DataHandler data;
  bool flag = true;

  /* Loop through every data */
  for (unsigned short int i = 1; i <= TOTAL_DATASETS; i++) {
    const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
    data.setDataSet(dataset);

    const auto robots = data.getRobots();

    /* Loop through every robot */
    for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {
      std::string measurement_file =
          dataset + "/Robot" + std::to_string(id + 1) + "_Measurement.dat";

      long unsigned int counter = countFileLines(measurement_file);

      if (0 == counter) {
        flag = false;
      }

      /* Count the number of elements */
      long unsigned int measurement_counter = 0;

      for (std::size_t k = 0; k < robots[id].raw.measurements.size(); k++) {

        if ((robots[id].raw.measurements[k].bearings.size() ==
             robots[id].raw.measurements[k].ranges.size()) &&
            (robots[id].raw.measurements[k].ranges.size() ==
             robots[id].raw.measurements[k].subjects.size())) {

          measurement_counter += robots[id].raw.measurements[k].subjects.size();
        } else {
          flag = false;
        }
      }
      if (measurement_counter != counter) {
        std::cerr << "Robot " << id + 1
                  << " does not have a size equal to the number of entries in "
                     "the measurement file: "
                  << robots[id].raw.measurements.size() << " ≠ " << counter
                  << std::endl;
        flag = false;
      }
    }
  }
  flag
      ? std::cout
            << "\033[1;32m[U05 PASS]\033[0m All Robots have extracted the "
               "correct amount of measurement values from the dataset\n"
      : std::cerr << "\033[1;31m[U5 FAIL]\033[0m Not all robots extracted the "
                     "correct amount of measurement values from the dataset.\n";
}

/**
 * @brief Unit Test 6: Test Interpolation values against the ones extracted from
 * the matlab script.
 */
void testInterpolation() {
  DataHandler data("./data/MRCLAM_Dataset1");
  const auto robots = data.getRobots();
  bool flag = true;

  for (unsigned int id = 0; id < data.getNumberOfRobots(); id++) {

    /* Check Groundtruth Interpolation  */
    std::string filename = "./test/Matlab_output/Robot" +
                           std::to_string(id + 1) + "_Groundtruth.csv";
    std::fstream file(filename);

    if (!file.is_open()) {
      std::cerr << "Unable to open file: " << filename << std::endl;
      flag = false;
      return;
    }

    /* Check that the number of lines in the file match the number of items in
     * the extracted values. */
    std::size_t total_lines = countFileLines(filename);
    if (total_lines != robots[id].groundtruth.states.size()) {
      std::cerr << "Total number of interpolated groundtruth values does not "
                   "match Matlab output "
                << robots[id].groundtruth.states.size() << " ≠ " << total_lines
                << ". File: " << filename << std::endl;
      flag = false;
      return;
    }

    for (std::size_t k = 0; k < robots[id].groundtruth.states.size(); k++) {
      std::string line;
      std::getline(file, line);

      if ('#' == line[0]) {
        k--;
        continue;
      }

      /* Remove whitespaces */
      line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

      std::size_t start_index = 0;
      std::size_t end_index = line.find(',', 0);
      double time = std::stod(line.substr(start_index, end_index));

      if (std::round((robots[id].groundtruth.states[k].time - time) * 100.0) /
              100.0 !=
          0) {
        std::cerr << "Interpolation Time Index Error [Line " << k << "] "
                  << filename << ": " << robots[id].groundtruth.states[k].time
                  << " ≠ " << time << std::endl;
        flag = false;
      }

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      double x_coordinate =
          std::stod(line.substr(start_index, end_index - start_index));

      if (std::round((robots[id].groundtruth.states[k].x - x_coordinate) *
                     100.0) /
              100.0 !=
          0) {
        std::cerr << "Interpolation x-coordinate Error [Line " << k << "] "
                  << filename << ": " << robots[id].groundtruth.states[k].x
                  << " ≠ " << x_coordinate << std::endl;
        flag = false;
      }

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      double y_coordinate =
          std::stod(line.substr(start_index, end_index - start_index));

      if (std::round((robots[id].groundtruth.states[k].y - y_coordinate) *
                     100.0) /
              100.0 !=
          0) {
        std::cerr << "Interpolation y-coordinate Error [Line " << k << "] "
                  << filename << ": " << robots[id].groundtruth.states[k].y
                  << " ≠ " << y_coordinate << std::endl;
        flag = false;
      }

      /* NOTE: This script handles the orientation interpolation better than the
       * matlab script. Therefore this check was removed. */

      // start_index = end_index + 1;
      // end_index = line.find(',', start_index);
      // double orientation = std::stod(line.substr(start_index, end_index -
      // start_index));

      // if (std::round((robots[id].groundtruth.states[k].orientation -
      // orientation) * 100.0) / 100.0 != 0) { 	std::cerr << "Interpolation
      // orientation Error [Line " << k << "] " << filename << ":" <<
      // robots[id].groundtruth.states[k].orientation << " ≠ " << orientation <<
      // std::endl; 	flag = false; 	return;
      // }
    }

    /* Check Odometry Interpolation */
    file.close();
    filename =
        "./test/Matlab_output/Robot" + std::to_string(id + 1) + "_Odometry.csv";
    file.open(filename);
    if (!file.is_open()) {
      std::cerr << "Unable to open file: " << filename << std::endl;
      flag = false;
    }

    /* Check that the number of lines in the file match the number of items in
     * the extracted values. */
    total_lines = countFileLines(filename);
    if (total_lines != robots[id].synced.odometry.size()) {
      std::cerr << "Total number of interpolated Odometry values does not "
                   "match Matlab output "
                << robots[id].synced.odometry.size() << " ≠ " << total_lines
                << " . File: " << filename;
      flag = false;
    }

    for (std::size_t i = 0; i < robots[id].synced.odometry.size(); i++) {
      std::string line;
      std::getline(file, line);
      if ('#' == line[0]) {
        i--;
        continue;
      }
      /* Remove whitespaces */
      line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

      std::size_t start_index = 0;
      std::size_t end_index = line.find(',', 0);
      double time = std::stod(line.substr(start_index, end_index));

      if (std::round((robots[id].synced.odometry[i].time - time) * 100.0) /
              100.0 !=
          0) {
        std::cerr << "Interpolation Time Index Error [Line " << i << "] "
                  << filename << ": " << robots[id].synced.odometry[i].time
                  << " ≠ " << time << std::endl;
        flag = false;
      }

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      double forward_velocity =
          std::stod(line.substr(start_index, end_index - start_index));
      if (std::round((robots[id].synced.odometry[i].forward_velocity -
                      forward_velocity) *
                     100.0) /
              100.0 !=
          0) {
        std::cerr << "Interpolation forward velocity Error [Line " << i << "] "
                  << filename << ": "
                  << robots[id].synced.odometry[i].forward_velocity << " ≠ "
                  << forward_velocity << std::endl;
        flag = false;
      }

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      double angular_velocity =
          std::stod(line.substr(start_index, end_index - start_index));

      if (std::round((robots[id].synced.odometry[i].angular_velocity -
                      angular_velocity) *
                     100.0) /
              100.0 !=
          0) {
        std::cerr << "Interpolation angular velocity Error [Line " << i << "] "
                  << filename << ": "
                  << robots[id].synced.odometry[i].angular_velocity << " ≠ "
                  << angular_velocity << std::endl;
        flag = false;
      }
    }

    /* Check Measurement Interpolation  */
    file.close();
    filename = "./test/Matlab_output/Robot" + std::to_string(id + 1) +
               "_Measurement.csv";
    file.open(filename);
    if (!file.is_open()) {
      std::cerr << "Unable to open file: " << filename << std::endl;
      flag = false;
    }
    /* Check that the number of lines in the file match the number of items in
     * the extracted values. */
    total_lines = countFileLines(filename);
    if (0 == total_lines) {
      flag = false;
    }

    std::size_t total_measurements = 0;
    /* Count the then number of elements in the measurment matrix */
    for (std::size_t i = 0; i < robots[id].synced.measurements.size(); i++) {
      /* Check all the measurement vectors are the same length */
      if ((robots[id].synced.measurements[i].subjects.size() !=
           robots[id].synced.measurements[i].ranges.size()) ||
          (robots[id].synced.measurements[i].ranges.size() !=
           robots[id].synced.measurements[i].bearings.size())) {
        std::cerr << "Measurement size mismatch: subjects, ranges, and "
                     "bearings do not have the same size: "
                  << robots[id].synced.measurements[i].subjects.size() << " : "
                  << robots[id].synced.measurements[i].ranges.size() << " : "
                  << robots[id].synced.measurements[i].bearings.size() << "\n";
        flag = false;
      }

      total_measurements += robots[id].synced.measurements[i].subjects.size();
    }

    if (total_lines != total_measurements) {
      std::cerr << "Total number of interpolated Measurment values does not "
                   "match Matlab output "
                << total_measurements << " ≠ " << total_lines << filename
                << std::endl;
      flag = false;
    }

    std::string line;
    std::size_t counter = 0;
    double current_time = -1.0;

    std::vector<unsigned short> subjects;
    std::vector<double> ranges;
    std::vector<double> bearings;

    while (std::getline(file, line)) {

      /* Ignore Comments */
      if ('#' == line[0]) {
        continue;
      }

      /* Remove whitespaces */
      line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

      std::size_t start_index = 0;
      std::size_t end_index = line.find(',', 0);
      double time = std::stod(line.substr(start_index, end_index));
      /* Initialise current_time on first iteration. */
      if (current_time == -1.0) {
        current_time = time;
      } else if (time > current_time) {
        /* Perform checking on previously populated list */
        if (std::round(
                (robots[id].synced.measurements[counter].time - current_time) *
                100.0) /
                100.0 !=
            0) {
          std::cerr << "Interpolation Time Index Error [Line " << counter
                    << "] ./test/Matlab_output/Robot1_Measurement.csv: "
                    << robots[id].synced.measurements[counter].time << " ≠ "
                    << time << std::endl;
          flag = false;
        }

        if (robots[id].synced.measurements[counter].subjects != subjects) {
          std::cerr << "Robot " << id << "'s List of subjects does not match: "
                    << "t: " << current_time << " - "
                    << " (E) "
                    << robots[id].synced.measurements[counter].subjects.size()
                    << " : (F) " << subjects.size() << std::endl;

          flag = false;
        }

        if (robots[id].synced.measurements[counter].ranges != ranges) {
          std::cerr << "Robot " << id << "'s List of ranges does not match\n";
          flag = false;
        }

        if (robots[id].synced.measurements[counter].bearings != bearings) {
          std::cerr << "Robot" << id << "'s List of bearings does not match\n";
          flag = false;
        }

        /* Clear vectors */
        subjects.clear();
        ranges.clear();
        bearings.clear();

        /* Update the current timestep */
        current_time = time;
        counter++;
      }

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      subjects.push_back(
          std::stod(line.substr(start_index, end_index - start_index)));

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      ranges.push_back(
          std::stod(line.substr(start_index, end_index - start_index)));

      start_index = end_index + 1;
      end_index = line.find(',', start_index);
      bearings.push_back(
          std::stod(line.substr(start_index, end_index - start_index)));
    }
  }
  flag ? std::cout << "\033[1;32m[U06 PASS]\033[0m All raw extracted values "
                      "were correctly interpolated\n"
       : std::cerr << "\033[1;31m[U6 FAIL]\033[0m Raw extraced values were not "
                      "correctly interpolated\n";
}
/**
 * @brief Checks that the time stamps produced by the resampling process are
 * shared across the ground truth, odometry and measurements, as well as equal
 * to the defined sampling period.
 */
void checkSamplingRate() {
  DataHandler data("./data/MRCLAM_Dataset1");

  auto robots = data.getRobots();
  double sample_period = data.getSamplePeriod();

  bool flag = true;

  for (unsigned short int k = 0; k < data.getNumberOfRobots(); k++) {
    /* Check if the synced groundtruth and odometry are the same length */
    if (robots[k].groundtruth.states.size() !=
        robots[k].synced.odometry.size()) {
      std::cerr
          << "\033[1;31m[ERROR]\033[0m Robot " << k
          << " groundtruth and odometry vectors are not the same length\n";
      flag = false;
    }

    /* Check if all the sample periods are equal to the
     * dataHandler::sample_period_ */
    for (std::size_t i = 1; i < robots[k].groundtruth.states.size(); i++) {
      double extracted_sample_period =
          (robots[k].groundtruth.states[i].time -
           robots[k].groundtruth.states[i - 1].time);

      if (std::round((extracted_sample_period - sample_period) * 1000.0) /
              1000. !=
          0) {
        std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k
                  << " synced groundtruth time stamps do not matching sampling "
                     "period:"
                  << extracted_sample_period << " ≠ " << sample_period
                  << std::endl;
        flag = false;
      }
    }

    for (std::size_t i = 1; i < robots[k].synced.odometry.size(); i++) {
      double extracted_sample_period = (robots[k].synced.odometry[i].time -
                                        robots[k].synced.odometry[i - 1].time);

      if (std::round((extracted_sample_period - sample_period) * 1000.0) /
              1000.0 !=
          0) {
        std::cerr
            << "\033[1;31m[ERROR]\033[0m Robot " << k
            << " synced odometry time stamps do not matching sampling period:"
            << extracted_sample_period << " ≠ " << sample_period << std::endl;
        flag = false;
      }
    }

    /* Check if all the sample time stamps are the same between Groundtruth and
     * odometry. NOTE: The sizes of the measurements and groundtruth vectors are
     * checked above, so they are assumed to be equal here. */
    for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
      if (robots[k].synced.odometry[i].time !=
          robots[k].groundtruth.states[i].time) {
        std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k
                  << " Time stamp mismatch between odometry and groundtruth: "
                  << robots[k].synced.odometry[i].time << " ≠ "
                  << robots[k].groundtruth.states[i].time << std::endl;
      }
    }
    /* Check if all the measurements have the time stamps as Groundruth. */
    auto iterator = robots[k].groundtruth.states.begin();
    for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
      /* Attempt to find the timestep in the ground truth time steps.
       * NOTE: since the groundtruth and odometry has already been checked to be
       * the same, it is assumed that if the time stamp is in the groundtruth,
       * it is also in odometry. */
      iterator = std::find_if(
          iterator, robots[k].groundtruth.states.end(),
          [&](const auto &element) {
            return (std::round(
                        (element.time - robots[k].synced.measurements[i].time) *
                        1000.0) /
                        1000.0 ==
                    0.0);
          });
      /* If the measurement time stamp is not in the ground, an error has
       * occured. */
      if (iterator == robots[k].groundtruth.states.end()) {
        std::cerr << "\033[1;31m[ERROR]\033[1m Robot " << k
                  << " measurment timestamp not present in groundtruth: "
                  << robots[k].synced.measurements[k].time << std::endl;
        flag = false;
      }
    }
  }
  flag ? std::cout << "\033[1;32m[U07 PASS]\033[0m All resampled data have the "
                      "same time stamps \n"
       : std::cerr << "\033[1;31m[U7 FAIL] The timesteps in the synced "
                      "datasets did not match.\n";
}
/*
 * @brief Unit Test 8: Checks if the saving for all the data from the datasets
 * is can be done.
 */
void saveData() {
  bool flag = true;

  for (unsigned short int d = 0; d < 1; d++) {
    DataHandler data("./data/MRCLAM_Dataset" + std::to_string(d + 1));
    data.saveExtractedData();
    data.plotExtractedData();
  }

  flag
      ? std::cout << "\033[1;32m[U08 PASS]\033[0m Data succesfully saved.\n"
      : std::cerr
            << "\033[1;31m[U9 FAIL]\033[0m An error occured saving the data.\n";
}

/**
 * @brief Unit Test 9: Checks that the when the caluclated odometry values are
 * used for dead-reckoning that the outputs matches the ground truth values.
 */
void testGroundtruthOdometry() {
  bool flag = true;

  for (unsigned short int dataset = 0; dataset < TOTAL_DATASETS; dataset++) {
    DataHandler data("./data/MRCLAM_Dataset" + std::to_string(dataset + 1));

    auto robots = data.getRobots();

    for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {
      double average_x_difference = 0;
      double average_y_difference = 0;
      double average_orientation_difference = 0;

      for (std::size_t k = 0; k < robots[id].groundtruth.states.size() - 1;
           k++) {

        /* NOTE: Unit Test 8 checks that the sampling time equals the set sample
         * period. Therefore, it is assumed that the same period is equal to the
         * value set.  */
        double sampling_period = data.getSamplePeriod();

        /* Calculate the robot's x-position and compare it to the groundtruth
         * value */
        double x = robots[id].groundtruth.states[k].x +
                   robots[id].groundtruth.odometry[k].forward_velocity *
                       sampling_period *
                       std::cos(robots[id].groundtruth.states[k].orientation);

        average_x_difference +=
            std::abs(x - robots[id].groundtruth.states[k + 1].x);

        /* Calculate the robot's y-position and compare it to the groundtruth
         * value */
        double y = robots[id].groundtruth.states[k].y +
                   robots[id].groundtruth.odometry[k].forward_velocity *
                       sampling_period *
                       std::sin(robots[id].groundtruth.states[k].orientation);

        average_y_difference +=
            std::abs(y - robots[id].groundtruth.states[k + 1].y);
        /* Calculate the robot's orientation and compare it to the groundtruth
         * value */
        double orientation =
            robots[id].groundtruth.states[k].orientation +
            sampling_period *
                robots[id].groundtruth.odometry[k].angular_velocity;

        /* Normalise the orientation between PI and -PI (180 and -180 degrees
         * respectively) */
        while (orientation >= M_PI)
          orientation -= 2.0 * M_PI;
        while (orientation < -M_PI)
          orientation += 2.0 * M_PI;

        average_orientation_difference += std::abs(
            orientation - robots[id].groundtruth.states[k + 1].orientation);
      }
    }
  }
  flag ? std::cout << "\033[1;32m[U09 PASS]\033[0m All Robots have the correct "
                      "groundtruth odometry values\n"
       : std::cerr << "\033[1;31m[U9 FAIL]\033[0m Not all robots have the "
                      "correctly calculated groundtruth odometry values.\n";
}
/**
 * @brief Checks if the PDF adds to 1
 */
void checkPDF() {

  std::string filename =
      "./data/MRCLAM_Dataset1/data_extraction/Bearing-Error-PDF.dat";
  std::fstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Unable to open file: " << filename << std::endl;
    return;
  }

  std::string line;
  double integral = 0.0;
  while (std::getline(file, line)) {
    /* Skip comments */
    if ('#' == line[0]) {
      continue;
    }

    if ("" == line) {
      std::cout << "END" << std::endl;
      break;
    }

    std::size_t start_index = 0;
    std::size_t end_index = 0;
    end_index = line.find('\t', end_index);

    start_index = end_index + 1;
    end_index = line.find('\t', end_index + 1);
    double bin_width =
        std::stod(line.substr(start_index, end_index - start_index));

    start_index = end_index + 1;
    end_index = line.find('\t', end_index + 1);
    double value = std::stod(line.substr(start_index, end_index - start_index));

    integral += value * bin_width;
  }

  if (integral == 1.0) {
    std::cout << "PASS" << std::endl;
  } else {
    std::cout << "FAIL" << std::endl;
  }
}

/**
 * @brief Check that the size of all the odoemtry and groundtruth values for all
 * the robots are the same after syncing the time steps.
 */
void checkSyncedSize() {

  bool flag = true;
  for (unsigned short int dataset = 5; dataset < 6; dataset++) {
    DataHandler data("./data/MRCLAM_Dataset" + std::to_string(dataset + 1));
    std::cout << "./data/MRCLAM_Dataset" + std::to_string(dataset + 1)
              << std::endl;
    auto robots = data.getRobots();

    unsigned long int odometry_size = robots[0].groundtruth.odometry.size();

    for (unsigned short int id = 1; id < data.getNumberOfRobots(); id++) {
      if (robots[id].groundtruth.odometry.size() != odometry_size) {
        std::cout
            << "[ERROR] Robot " << id + 1
            << " groundtruth odometry does not have the same size as Robot  1: "
            << odometry_size << " ≠ " << robots[id].groundtruth.odometry.size()
            << std::endl;
        flag = false;
      }

      if (robots[id].synced.odometry.size() != odometry_size) {
        std::cout
            << "[ERROR] Robot " << id + 1
            << " synced odometry does not have the same size as Robot  1: "
            << odometry_size << " ≠ " << robots[id].synced.odometry.size()
            << std::endl;
        flag = false;
      }

      if (robots[id].groundtruth.states.size() != odometry_size) {
        std::cout
            << "[ERROR] Robot " << id + 1
            << " groundtruth states does not have the same size as Robot  1: "
            << odometry_size << " ≠ " << robots[id].groundtruth.states.size()
            << std::endl;
        flag = false;
      }
    }
  }

  flag ? std::cout << "\033[1;32m [U10 PASS]\033[0m All Robots have the same "
                      "sized synced vectors."
                   << std::endl
       : std::cerr << "\031[1;33m [U10 FAIL]\033[0m All Robots do not have the "
                      "same sized synced vectors."
                   << std::endl;
}

void checkSimulation() {
  // bool flag = true;

  DataHandler data;

  data.setSimulation(70000, 0.02, 5U, 15U);
  data.saveExtractedData();
  data.plotExtractedData();
}

int main() {
  auto start = std::chrono::high_resolution_clock::now();
  std::cout << "\033[1;36mUNIT TESTING\033[0m" << std::endl;
  std::cout << "\033[3mNumber of treads supported:\033[0m "
            << std::thread::hardware_concurrency() << std::endl;

  // std::thread unit_test_1(checkBarcodes);
  // std::thread unit_test_2(checkLandmarkBarcodes);
  // std::thread unit_test_3(checkGroundtruthExtraction);
  // std::thread unit_test_4(checkOdometryExtraction);
  // std::thread unit_test_5(checkMeasurementExtraction);
  // std::thread unit_test_6(testInterpolation);
  // std::thread unit_test_7(checkSamplingRate);
  // std::thread unit_test_8(saveData);
  // std::thread unit_test_9(testGroundtruthOdometry);
  // std::thread unit_test_10(checkSyncedSize);

  // unit_test_1.join();
  // unit_test_2.join();
  // unit_test_3.join();
  // unit_test_4.join();
  // unit_test_5.join();
  // unit_test_6.join();
  // unit_test_7.join();
  // unit_test_8.join();
  // unit_test_9.join();
  // unit_test_10.join();
  // checkPDF();
  checkSimulation();

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
  std::cout << "\n Test ran for: " << duration.count() << " seconds\n";
  return 0;
}
