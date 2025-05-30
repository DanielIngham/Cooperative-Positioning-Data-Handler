/**
 * The class extracts the data from the groundtruth states, measured range and
 * bearings, and odometry. These values are used to populate three main data
 * structs: Barcodes, Landmarks, and Robots.
 * @file DataHandler.cpp
 * @brief Class implementation file responsible for exracting the ground-truth,
 * odometry and measurement data from the UTIAS multi-robot localisation
 * dataset.
 * @author Daniel Ingham
 * @date 2025-03-28
 */

#include "DataHandler.h"

#include <algorithm>  // std::remove_if and std::find
#include <chrono>     // std::chrono
#include <cstdlib>    // std::getenv
#include <filesystem> // std::filesystem
#include <fstream>    // std::ifstream
#include <iostream>   // std::cout
#include <sstream>    // std::ostringstream
#include <stdexcept>  // std::runtime_error
#include <string>
#include <unordered_map> // std::unordered_map
/**
 * @brief Default constructor.
 */
DataHandler::DataHandler() {}

/**
 * @brief Constructor that sets the simuation values for the multi-robot
 * localisation and mapping.
 * @param[in] data_points The number of timestep to be simulated.
 * @param[in] sample_period The period at which the odometry sensor is sampled.
 * @param[in] number_of_robots The total number of robots to be simulated.
 * @param[in] number_of_landmarks The total number of landmarks to be simulated.
 * @param[in] output_directory The directory where the extracted data and plots
 * are saved.
 */
DataHandler::DataHandler(const unsigned long int data_points,
                         double sample_period,
                         const unsigned short number_of_robots,
                         const unsigned short number_of_landmarks,
                         const std::string &output_directory)
    : sampling_period_(sample_period), total_landmarks(number_of_landmarks),
      total_robots(number_of_robots),
      total_barcodes(total_landmarks + total_robots),
      total_synced_datapoints(data_points), landmarks_(total_landmarks),
      robots_(total_robots), barcodes_(total_barcodes) {

  setSimulation(data_points, sample_period, number_of_robots,
                number_of_landmarks, output_directory);
}

/**
 * @brief Constructor that extracts and populates class attributes using the
 * values from the dataset provided.
 * @param[in] dataset directory path to the dataset folder.
 * @param[in] sample_period the desired sample period for resampling the data to
 * sync the timesteps between the vehicles.
 * @param[in] output_directory The directory where the extracted data and plots
 * are saved.
 * @note The dataset extractor constructor only takes one dataset at at time.
 */
DataHandler::DataHandler(const std::string &dataset,
                         const std::string &output_directory,
                         const double &sample_period)
    : dataset_(dataset), output_directory_(output_directory),
      sampling_period_(sample_period), total_landmarks(15), total_robots(5),
      total_barcodes(total_landmarks + total_robots),
      landmarks_(total_landmarks), robots_(total_robots),
      barcodes_(total_barcodes, 0) {

  setDataSet(dataset, output_directory, sample_period);
}

/**
 * @brief Creates simulation values for the robots and landmarks.
 * @param[in] data_points The number of timestep to be simulated.
 * @param[in] sample_period The period at which the odometry sensor is sampled.
 * @param[in] number_of_robots The total number of robots to be simulated.
 * @param[in] number_of_landmarks The total number of landmarks to be simulated.
 * @param[in] output_directory The directory where the extracted data and plots
 * are saved.
 */
void DataHandler::setSimulation(const unsigned long int data_points,
                                double sample_period,
                                const unsigned short number_of_robots,
                                const unsigned short number_of_landmarks,
                                const std::string &output_directory) {

  auto start = std::chrono::high_resolution_clock::now();
  /* Set class fields */
  this->dataset_ = "./";

  setOutputDirectory(output_directory, "/simulation/");

  /* Set class fields. */
  this->total_synced_datapoints = data_points;

  this->sampling_period_ = sample_period;

  this->total_landmarks = number_of_landmarks;
  this->total_robots = number_of_robots;
  this->total_barcodes = total_landmarks + total_robots;

  /* Resize the dataset vectors */
  this->landmarks_.resize(total_landmarks);
  this->robots_.resize(total_robots);
  this->barcodes_.resize(total_barcodes, 0);

  simulator.setSimulation(data_points, sample_period, robots_, landmarks_,
                          barcodes_);
  try {
    /* Calculate odometry and measurement errors. */
    for (int i = 0; i < total_robots; i++) {
      robots_[i].calculateSensorErrror();
    }

    /* Stop timer after extraction. */
    auto end = std::chrono::high_resolution_clock::now();

    /* Calculate duration. */
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "\033[1;32mSimulation Complete:\033[0m \033[3m"
              << this->data_extraction_directory_ << "\033[0m ["
              << duration.count() << " ms]" << std::endl;
  } catch (std::runtime_error &error) {
    std::cerr << "Unable to calculate error statistics: " << error.what()
              << std::endl;
    throw;
  }
}

/**
 * @brief Extracts data from the all files in the specified dataset folder.
 * @param[in] dataset dataset path to the dataset folder.
 * @param[in] sample_period the desired sample period for resampling the data to
 * @param[in] output_directory The directory where the extracted data and plots
 * are saved.
 * sync the timesteps between the vehicles.
 * @note All datasets in the UTIAS Multi-robot Localisation and mapping dataset
 * have the same number of landmarks and robots. Therefore, if the dataset
 * directory is provided, it is assumed that the number of landmarks and robots
 * will be 5 and 15 respectively. The number of barcodes is the summation and
 * these two values.
 * @note The function only checks the existence of the given datset folder. The
 * data extraction is performed by calling the functions:
 * DataHandler::readBarcodes, DataHandler::readLandmarks,
 * DataHandler::readGroundTruth, DataHandler::readOdometry, and
 * DataHandler::readMeasurements. Additionally, the DataHandler::syncData
 * function is called to resample to data points through linear interpolation to
 * ensure all robots have the same time stamps.
 */
void DataHandler::setDataSet(const std::string &dataset,
                             const std::string &output_directory,
                             const double &sample_period) {
  /* Start timer for measurement of extraction period. */
  auto start = std::chrono::high_resolution_clock::now();

  /* Check if the data set directory exists */
  this->dataset_ = LIB_DIR + ("/data/" + dataset);

  if (!std::filesystem::exists(dataset_)) {
    throw std::runtime_error("Dataset file path does not exist: " + dataset_);
  }

  setOutputDirectory(output_directory, dataset);

  /* Set the sample period for this dataset. */
  this->sampling_period_ = sample_period;

  /* All datasets contain 15 landmarks and 5 robots. */
  this->total_landmarks = 15U;
  this->total_robots = 5U;
  this->total_barcodes = total_landmarks + total_robots;

  /* Resize the dataset vectors */
  this->landmarks_.resize(total_landmarks);
  this->robots_.resize(total_robots);
  this->barcodes_.resize(total_barcodes, 0);

  /* Set the robot ID */
  for (unsigned short int id = 0; id < total_robots; id++) {
    robots_[id].id = id + 1;
  }

  try {
    /* Perform data extraction in the directory */
    readBarcodes(dataset_);
    readLandmarks(dataset_);

    /* Populate the values for each robot from the dataset */
    for (int id = 0; id < total_robots; id++) {
      readGroundTruth(dataset_, id);
      readOdometry(dataset_, id);
      readMeasurements(dataset_, id);
    }

  } catch (std::runtime_error &error) {
    std::cerr << "\033[1;32mUnable to extract data from " << dataset
              << ":\033[0m " << error.what();
    /* Re-throw the same error. */
    throw;
  }

  /* Perform Time Stamp Synchronisation. This performs the linear interpolations
   * of the values — ensuring all values have the same time steps  */
  syncData(sample_period);

  /* Calculate the odometry values that would correspond to the ground truth
   * position and heading values after synchronsation. */
  calculateGroundtruthOdometry();

  /* Calculate the measurement values that would correspond to the ground truth
   * range and bearing values. */
  calculateGroundtruthMeasurement();

  try {
    /* Calculate odometry and measurement errors. */
    for (int i = 0; i < total_robots; i++) {
      robots_[i].calculateSensorErrror();
      robots_[i].calculateSampleErrorStats();
    }
    /* Stop timer after extraction. */
    auto end = std::chrono::high_resolution_clock::now();
    /* Calculate duration. */
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "\033[1;32mData Extraction Complete:\033[0m \033[3m" << dataset
              << "\033[0m [" << duration.count() << " ms]" << std::endl;
  } catch (std::runtime_error &error) {
    std::cerr << "Unable to calculate error statistics: " << error.what()
              << std::endl;
    throw;
  }
}

/**
 * @brief Sets the output directory for the data plots.
 * @param[in] output_directory The output directory.
 * @param[in] folder The name of folder within the output_directory.
 */
void DataHandler::setOutputDirectory(const std::string &output_directory,
                                     const std::string &folder) {

  this->output_directory_ =
      std::getenv("PROJECT_DIR") + ("/output/" + output_directory);

  /* Creates unique simulation folder using the current system time. */
  try {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm now_tm = *std::localtime(&now_c);
    std::ostringstream oss;

    oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");

    std::string unique_directory =
        output_directory_ + '/' + folder + '/' + oss.str();

    this->data_extraction_directory_ = unique_directory + "/data_extraction/";
    this->data_inference_directory = unique_directory + "/inference";

  } catch (std::runtime_error &error) {
    std::cout << "Unable to set dataset: " << error.what() << std::endl;
    throw;
  }
}

/**
 * @brief Extracts data from the barcodes data file: Barcodes.dat.
 * @param[in] dataset directory path to the dataset folder.
 * @note If the data could not be extracted from the specified dataset, a
 * std::runtime_error is thrown.
 */
void DataHandler::readBarcodes(const std::string &dataset) {
  /* Check that the dataset was specified */
  if ("" == this->dataset_) {
    throw std::runtime_error(
        "Dataset not specified. Please specify a dataset at construction of "
        "the DataHandler class instance or using DataHandler::setDataSet.");
  }

  std::string filename = dataset + "/Barcodes.dat";
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Unable to open barcodes file: " + filename);
  }

  /* Iterate through file line by line.*/
  std::string line;
  int i = 0;

  while (std::getline(file, line)) {
    /* Ignore file comments. */
    if (line[0] == '#') {
      continue;
    }

    /* Remove whitespaces */
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

    if (i >= total_barcodes) {
      throw std::runtime_error("The number of barcodes read exceeds total "
                               "number of barcodes specified.");
    }

    if (barcodes_.size() == 0) {
      throw std::runtime_error(
          "The total number of barcodes was not specified.");
    }

    /* Extract barcodes into barcodes array */
    barcodes_[i] = std::stoi(line.substr(line.find('\t', 0)));
    if (i < total_robots) {
      robots_[i].barcode = barcodes_[i];
    } else {
      landmarks_[i - total_robots].barcode = barcodes_[i];
    }
    i++;
  }

  file.close();
}
/**
 * @brief Extracts data from the landmarks data file: Landmark_Groundtruth.dat.
 * @param[in] dataset path to the dataset folder.
 * @note DataHandler::readBarcodes needs to be called before this function since
 * this function relies on the barcodes extracted.
 * @note If the data could not be extracted from the specified dataset, a
 * std::runtime_error is thrown.
 */
void DataHandler::readLandmarks(const std::string &dataset) {

  std::string filename = dataset + "/Landmark_Groundtruth.dat";
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Unable to open Landmarks file: " + filename);
  }
  /* Iterate through file line by line.*/
  int i = 0;
  std::string line;

  while (std::getline(file, line)) {
    /* Ignore file comments. */
    if ('#' == line[0]) {
      continue;
    }

    /* Remove whitespaces */
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

    if (i >= total_landmarks) {
      throw std::runtime_error(
          "Total number of read landmarks exceeds TOTAL_LANDMARKS variable.\n");
    }

    /* Set the landmark's ID */
    std::size_t start_index = 0;
    std::size_t end_index = line.find('\t', 0);
    landmarks_[i].id = std::stoi(line.substr(start_index, end_index));

    /* Ensure that the barcodes have been extracted and set */
    if (barcodes_[landmarks_[i].id - 1] == 0) {
      throw std::runtime_error("An error occured with barcodes extraction, "
                               "barcodes were not correctly set.");
    }

    /* Set landmark's barcode */
    landmarks_[i].barcode = barcodes_[landmarks_[i].id - 1];

    /* Landmark x-coordinate [m] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    landmarks_[i].x =
        std::stod(line.substr(start_index, end_index - start_index));

    /* Landmark y-coordinate [m] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    landmarks_[i].y =
        std::stod(line.substr(start_index, end_index - start_index));

    /* Landmark x standard deviation [m] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    landmarks_[i].x_std_dev =
        std::stod(line.substr(start_index, end_index - start_index));

    /* Landmark y standard deviation [m] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    landmarks_[i++].y_std_dev =
        std::stod(line.substr(start_index, end_index - start_index));
  }

  file.close();
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Groundtruth.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement
 * will be assigned to.
 * @details The data extracted form the Robotx_Groundtruth.dat contains the
 * timestamp [s], x coordinate [m], y coordinate [m], and orientation [rad] of
 * the robot x. These are used to populate the Robot::raw states member for a
 * given robot in DataHandler::robots_.
 */
void DataHandler::readGroundTruth(const std::string &dataset, int robot_id) {
  /* Clear all previous elements in the ground truth vector. */
  robots_[robot_id].raw.states.clear();

  /* Setup file for data extraction */
  std::string filename =
      dataset + "/Robot" + std::to_string(robot_id + 1) + "_Groundtruth.dat";
  std::ifstream file(filename);

  /* Check if the file could be opened */
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open groundtruth data file: " +
                             filename);
  }

  /* Loop through each line in the file. */
  std::string line;
  while (std::getline(file, line)) {
    /* Ignore Comments */
    if ('#' == line[0]) {
      continue;
    }

    /* Remove whitespaces */
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

    /* Extract Data into thier respective variables */
    /* - Time */
    std::size_t start_index = 0;
    std::size_t end_index = line.find('\t', 0);
    double time = std::stod(line.substr(start_index, end_index));

    /* - x-coordinate [m] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    double x_coordinate =
        std::stod(line.substr(start_index, end_index - start_index));

    /* - y-coordinate [m] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    double y_coordinate =
        std::stod(line.substr(start_index, end_index - start_index));

    /* - Orientaiton [rad] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    double orientation =
        std::stod(line.substr(start_index, end_index - start_index));

    /* Populate robot states with exracted values. */
    robots_[robot_id].raw.states.push_back(
        Robot::State(time, x_coordinate, y_coordinate, orientation));
  }

  file.close();
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Odometry.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement
 * will be assigned to.
 * @details The data extracted form the Robotx_Odometry.dat contains the
 * timestamp [s], Forward Velocity [m/s], and Angular velocity [rad/s] of the
 * measured odometry input into robot x. These are used to populate the
 * Robot::raw odometry member for a given robot in DataHandler::robots_.
 */
void DataHandler::readOdometry(const std::string &dataset, int robot_id) {
  /* Clear all previous elements in the odometry vector. */
  robots_[robot_id].raw.odometry.clear();

  /* Setup file for data extraction */
  std::string filename =
      dataset + "/Robot" + std::to_string(robot_id + 1) + "_Odometry.dat";
  std::fstream file(filename);

  /* Check if the file could be opened */
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open odometry data file: " + filename);
  }

  /* Loop through each line in the file. */
  std::string line;
  while (std::getline(file, line)) {
    /* Ignore Comments */
    if ('#' == line[0]) {
      continue;
    }
    /* Remove Whitespace */
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

    /* Extract Data into thier respective variables */
    /* - Time */
    std::size_t start_index = 0;
    std::size_t end_index = line.find('\t', 0);
    double time = std::stod(line.substr(start_index, end_index));

    /* - Forward Velocity [m/s] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    double forward_velocity =
        std::stod(line.substr(start_index, end_index - start_index));
    ;
    /* - Angular Velocity [rad/s] */
    start_index = end_index + 1;
    end_index = line.find('\t', start_index);
    double angular_velocity =
        std::stod(line.substr(start_index, end_index - start_index));
    ;
    /* Populate the robot class with the extracted values. */
    robots_[robot_id].raw.odometry.push_back(
        Robot::Odometry(time, forward_velocity, angular_velocity));
  }

  file.close();
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Measurement.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement
 * will be assigned to.
 * @note The data values are tab seperated.
 * @note Grouping of measurements with the same time stamps does not occur
 * during the reading. Therfore, the each member vector of measurements
 * (subjects, ranges and bearings) are filled with only one value. The grouping
 * by time stamp occurs in the DataHandler::syncData function.
 */
void DataHandler::readMeasurements(const std::string &dataset, int robot_id) {
  /* Clear all previous elements in the measurement vector. */
  robots_[robot_id].raw.measurements.clear();
  robots_[robot_id].synced.measurements.clear();

  /* Setup file for data extraction */
  std::string filename =
      dataset + "/Robot" + std::to_string(robot_id + 1) + "_Measurement.dat";
  std::fstream file(filename);

  /* Check if the file could be opened */
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open measurement data file: " +
                             filename);
  }

  /* Loop through each line in the file. */
  std::string line;
  while (std::getline(file, line)) {
    /* Ignore Comments */
    if ('#' == line[0]) {
      continue;
    }
    /* Remove Whitespaces */
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

    /* Extract Data into thier respective variables.  */
    /* - Time [s]*/
    std::size_t start_index = 0;
    std::size_t end_index = line.find('\t', 0);
    double time = std::stod(line.substr(start_index, end_index));

    /* - Subject (ID) */
    start_index = end_index;
    end_index = line.find('\t', ++end_index);
    int subject = std::stoi(line.substr(start_index, end_index));

    /* - Range [m] */
    start_index = end_index;
    end_index = line.find('\t', ++end_index);
    double range = std::stod(line.substr(start_index, end_index));

    /* - Bearing [rad] */
    start_index = end_index;
    end_index = line.find('\t', ++end_index);
    double bearing = std::stod(line.substr(start_index, end_index));

    robots_[robot_id].raw.measurements.push_back(
        Robot::Measurement(time, subject, range, bearing));
  }

  file.close();
}

/**
 * @brief Syncs the time steps for the extracted data according to the specified
 * sampling period.
 * @param[in] sample_period the desired sample period for resampling the data to
 * sync the timesteps between the vehicles.
 * @note Synced values for the ground truth are saved in the
 * DataHandler::robots_->groundtruth struct vector whereas synced odometry are
 * saved in the DataHandler::robots_->synced struct
 */
void DataHandler::syncData(const double &sample_period) {
  /* Find the minimum and maximimum times in the datasets */
  double minimum_time = robots_[0].raw.states.front().time;
  double maximum_time = robots_[0].raw.states.back().time;

  for (int i = 1; i < total_robots; i++) {
    double robot_minimum_time =
        std::min({robots_[i].raw.states.front().time,
                  robots_[i].raw.odometry.front().time,
                  robots_[i].raw.measurements.front().time});
    double robot_maximum_time = std::min(
        {robots_[i].raw.states.back().time, robots_[i].raw.odometry.back().time,
         robots_[i].raw.measurements.back().time});

    if (robot_minimum_time < minimum_time) {
      minimum_time = robot_minimum_time;
    }
    if (robot_maximum_time > maximum_time) {
      maximum_time = robot_maximum_time;
    }
  }

  /* Subtract the minimum time from all timesteps to make t=0 the intial time of
   * the system. */
  for (int i = 0; i < total_robots; i++) {
    /* Set the loop length to the size of the largest vector */
    std::size_t dataset_size =
        std::max({robots_[i].raw.states.size(), robots_[i].raw.odometry.size(),
                  robots_[i].raw.measurements.size()});

    for (std::size_t j = 0; j < dataset_size; j++) {
      if (j < robots_[i].raw.states.size()) {
        robots_[i].raw.states[j].time -= minimum_time;
      }
      if (j < robots_[i].raw.odometry.size()) {
        robots_[i].raw.odometry[j].time -= minimum_time;
      }
      if (j < robots_[i].raw.measurements.size()) {
        robots_[i].raw.measurements[j].time -= minimum_time;
      }
    }
  }

  maximum_time -= minimum_time;
  total_synced_datapoints = std::floor(maximum_time / sample_period) + 1;

  /* Linear Interpolation. This section performs linear interpolation on the
   * ground truth and odometry values to ensure that all robots have syncronised
   * time steps. */
  for (int id = 0; id < total_robots; id++) {
    /* Clear all previously interpolated values */
    robots_[id].groundtruth.states.clear();
    robots_[id].groundtruth.states.reserve(total_synced_datapoints);

    robots_[id].synced.odometry.clear();
    robots_[id].synced.odometry.reserve(total_synced_datapoints);

    robots_[id].synced.measurements.clear();

    auto groundtruth_iterator = robots_[id].raw.states.begin();
    auto odometry_iterator = robots_[id].raw.odometry.begin();

    for (double t = 0.0; t <= maximum_time; t += sample_period) {

      /* Find the first element that is larger than the current time step */
      groundtruth_iterator = std::find_if(
          groundtruth_iterator, robots_[id].raw.states.end(),
          [t](const Robot::State &element) { return element.time > t; });
      /* If the element is the first item in the raw values, copy the raw values
       * (no interpolation). This is assuming that the robot was stationary
       * before its ground truth was recorded. */
      if (groundtruth_iterator == robots_[id].raw.states.begin()) {
        robots_[id].groundtruth.states.push_back(
            Robot::State(t, robots_[id].raw.states.front().x,
                         robots_[id].raw.states.front().y,
                         robots_[id].raw.states.front().orientation));
        // continue;
      }

      /* If the element is the last item in the raw values, copy the raw values
         (no interpolation). This is assuming that the robot remains stationary
         after the ground truth recording ended. */
      else if (groundtruth_iterator == robots_[id].raw.states.end()) {
        robots_[id].groundtruth.states.push_back(Robot::State(
            t, robots_[id].raw.states.back().x, robots_[id].raw.states.back().y,
            robots_[id].raw.states.back().orientation));
        // continue;
      } else {
        /* Interpolate the Groundtruth values */
        double interpolation_factor =
            (t - (groundtruth_iterator - 1)->time) /
            (groundtruth_iterator->time - (groundtruth_iterator - 1)->time);

        double next_orientation = groundtruth_iterator->orientation;
        if (next_orientation - (groundtruth_iterator - 1)->orientation > 5) {
          next_orientation -= 2.0 * M_PI;
        } else if (next_orientation - (groundtruth_iterator - 1)->orientation <
                   -5) {
          next_orientation += 2.0 * M_PI;
        }

        next_orientation =
            interpolation_factor *
                (next_orientation - (groundtruth_iterator - 1)->orientation) +
            (groundtruth_iterator - 1)->orientation;

        /* Normalise the orientation between PI and -PI (180 and -180 degrees
         * respectively) */
        while (next_orientation >= M_PI)
          next_orientation -= 2.0 * M_PI;
        while (next_orientation < -M_PI)
          next_orientation += 2.0 * M_PI;

        robots_[id].groundtruth.states.push_back(Robot::State(
            t,
            interpolation_factor *
                    (groundtruth_iterator->x - (groundtruth_iterator - 1)->x) +
                (groundtruth_iterator - 1)->x,
            interpolation_factor *
                    (groundtruth_iterator->y - (groundtruth_iterator - 1)->y) +
                (groundtruth_iterator - 1)->y,
            next_orientation));
      }

      /* The same process as above is repeated for the odometry, except assume
       * the robot is stationary prior to ground truth readings */
      odometry_iterator = std::find_if(
          odometry_iterator, robots_[id].raw.odometry.end(),
          [t](const Robot::Odometry &element) { return element.time > t; });

      if (odometry_iterator == robots_[id].raw.odometry.begin() ||
          odometry_iterator == robots_[id].raw.odometry.end() - 1) {
        robots_[id].synced.odometry.push_back(Robot::Odometry(t, 0, 0));
        continue;
      }

      /* Calculating Odometry Interpolation */
      double interpolation_factor =
          (t - (odometry_iterator - 1)->time) /
          (odometry_iterator->time - (odometry_iterator - 1)->time);

      robots_[id].synced.odometry.push_back(Robot::Odometry(
          t,
          interpolation_factor * (odometry_iterator->forward_velocity -
                                  (odometry_iterator - 1)->forward_velocity) +
              (odometry_iterator - 1)->forward_velocity,
          interpolation_factor * (odometry_iterator->angular_velocity -
                                  (odometry_iterator - 1)->angular_velocity) +
              (odometry_iterator - 1)->angular_velocity));
    }

    /* The orginal UTIAS data extractor did NOT perform any linear interpolation
     * on the meaurement values. The only action that was performed on the
     * measurements was time stamp realignment according to the new timestamps.
     */
    robots_[id].synced.measurements.push_back(Robot::Measurement(
        std::floor(robots_[id].raw.measurements[0].time / sample_period + 0.5) *
            sample_period,
        robots_[id].raw.measurements[0].subjects,
        robots_[id].raw.measurements[0].ranges,
        robots_[id].raw.measurements[0].bearings));

    std::vector<Robot::Measurement>::iterator iterator =
        robots_[id].synced.measurements.end() - 1;

    /* Time stamp grouping: measurements with the same timestamps are grouped
     * together to improve accessability. */
    for (std::size_t j = 1; j < robots_[id].raw.measurements.size(); j++) {
      double synced_time =
          std::floor(robots_[id].raw.measurements[j].time / sample_period +
                     0.5) *
          sample_period;
      /* If the current measurment has the same time stamp the previous
       * measurment, join them. */
      if (synced_time == iterator->time) {
        iterator->subjects.push_back(
            robots_[id].raw.measurements[j].subjects[0]);
        iterator->ranges.push_back(robots_[id].raw.measurements[j].ranges[0]);
        iterator->bearings.push_back(
            robots_[id].raw.measurements[j].bearings[0]);
      } else {
        robots_[id].synced.measurements.push_back(Robot::Measurement(
            std::floor(robots_[id].raw.measurements[j].time / sample_period +
                       0.5) *
                sample_period,
            robots_[id].raw.measurements[j].subjects,
            robots_[id].raw.measurements[j].ranges,
            robots_[id].raw.measurements[j].bearings));
        iterator = robots_[id].synced.measurements.end() - 1;
      }
    }
  }
}

/**
 * @brief Utilises the extracted robots groundtruth position and heading values
 * to calculate their associated groundtruth odometry values.
 * @details The following expression is utilsed to calculate the groundtruth
 * odomotery values using the groundtruth states values extracted from the
 * dataset:
 * \f[\begin{bmatrix} \omega_k \\ v_k \end{bmatrix} = \begin{bmatrix}
 * \text{arctan2}(\sin(\theta_{k+1} - \theta_{k}), \cos(\theta_{k+1} -
 * \theta_{k})) / \Delta t \\ \sqrt{(x_{k+1} - x_k)^2 + (y_{k+1} - y_k)^2}
 * \end{bmatrix}, \f]
 * where \f$k\f$ denotes the current time step;
 * \f$\theta\f$ denotes the robot's orientation; \f$ y\f$ denotes the robot's
 * y-coordinate; \f$\Delta t\f$ is the user defined sample period; \f$\omega\f$
 * and \f$v\f$ denotes the angular velocity and forward velocity of the robot
 * respectively.
 */
void DataHandler::calculateGroundtruthOdometry() {
  for (int id = 0; id < total_robots; id++) {
    robots_[id].groundtruth.odometry.clear();

    for (std::size_t k = 0; k < robots_[id].groundtruth.states.size() - 1;
         k++) {

      double x_difference = (robots_[id].groundtruth.states[k + 1].x -
                             robots_[id].groundtruth.states[k].x);
      double y_difference = (robots_[id].groundtruth.states[k + 1].y -
                             robots_[id].groundtruth.states[k].y);

      robots_[id].groundtruth.odometry.push_back(Robot::Odometry(
          robots_[id].groundtruth.states[k].time,
          std::sqrt(x_difference * x_difference + y_difference * y_difference) /
              this->sampling_period_,
          std::atan2(
              std::sin(robots_[id].groundtruth.states[k + 1].orientation -
                       robots_[id].groundtruth.states[k].orientation),
              std::cos(robots_[id].groundtruth.states[k + 1].orientation -
                       robots_[id].groundtruth.states[k].orientation)) /
              this->sampling_period_));
    }
    /* NOTE: Since the last groundtruth odometry value can not be calculated, it
     * is set equal to the synced measured value */
    robots_[id].groundtruth.odometry.push_back(
        Robot::Odometry(robots_[id].synced.odometry.back().time,
                        robots_[id].synced.odometry.back().forward_velocity,
                        robots_[id].synced.odometry.back().angular_velocity));
  }
}

/**
 * @brief Calculates the ground truth measurements for a given robot.
 * @details The following expression is utilised to calculate the groundtruth
 * measurement values using the groundtruth robot state values extracted from
 * the dataset:
 * \f[\begin{bmatrix} r_{ij}^{(k)} \\ \phi_{ij}{(k)} \end{bmatrix} =
 * \begin{bmatrix} \sqrt{(x_i^{(k)} - x_j^{(k)})^2  + (y_i^{(k)}
 * - y_j^{(k)})^2} \\ \text{atan2}(y_j^{(k)} - y_i^{(k)}, x_j^{(k)} -
 * x_i^{(k)}) - \theta_i^{(k)}\end{bmatrix}, \f]
 * where \f$i\f$ denotes
 * the ego robot; \f$j\f$ denotes the measured robot; \f$k\f$ denotes the
 * current time step; \f$\theta\f$ denotes the robot's orientation; and \f$ y\f$
 * denotes the robot's y-coordinate.
 */
void DataHandler::calculateGroundtruthMeasurement() {
  for (int id = 0; id < total_robots; id++) {

    robots_[id].groundtruth.measurements.clear();
    auto iterator = robots_[id].groundtruth.measurements.begin();

    /* For loop iterator. Since the extracted data values ordered by time in
     * ascending order, once a time value is found, prior time values do not
     * need to be checked for newer time stamps. */
    size_t t = 0;

    for (std::size_t k = 0; k < robots_[id].synced.measurements.size(); k++) {
      /* Find the value of the ground truth with the same time stamp as the
       * measurement */
      for (; t < robots_[id].groundtruth.states.size(); t++) {
        if (std::round((robots_[id].groundtruth.states[t].time -
                        robots_[id].synced.measurements[k].time) *
                       1000.0) /
                1000.0 ==
            0.0) {
          break;
        }
      }

      /* Loop through each of the subjects and in the measurements and extract
       * the landmarks */
      for (std::size_t s = 0;
           s < robots_[id].synced.measurements[k].subjects.size(); s++) {
        /* Get the subjects ID from its barcode. */
        int subject_ID = getID(robots_[id].synced.measurements[k].subjects[s]);

        /* If the subjects barcode extracted does not correspond to any of the
         * barcodes extracted, then don't add the measurement. Then the ground
         * truth range and bearing measurments are set to zero. This is used by
         * the error calculator to determine if the measurement has a
         * corresponding groundtruth or not.*/
        double range = -1.0;         // Invalid range
        double bearing = 2.0 * M_PI; // Invalid Bearing

        if (-1 != subject_ID) {

          double x_difference;
          double y_difference;

          /* All robots have ID's [1,5]. */
          if (subject_ID < 6) {
            subject_ID--;
            x_difference = robots_[subject_ID].groundtruth.states[t].x -
                           robots_[id].groundtruth.states[t].x;
            y_difference = robots_[subject_ID].groundtruth.states[t].y -
                           robots_[id].groundtruth.states[t].y;
          }
          /* All landmarks have ID's [6,20]. */
          else {
            subject_ID -= 6;
            x_difference =
                landmarks_[subject_ID].x - robots_[id].groundtruth.states[t].x;
            y_difference =
                landmarks_[subject_ID].y - robots_[id].groundtruth.states[t].y;
          }

          /* Calculate Bearing */
          bearing = std::atan2(y_difference, x_difference) -
                    robots_[id].groundtruth.states[t].orientation;
          /* Normalise bearing between -180 and 180 (-pi and pi respectively)*/
          while (bearing >= M_PI)
            bearing -= 2.0 * M_PI;
          while (bearing < -M_PI)
            bearing += 2.0 * M_PI;

          /* Calculate Range */
          range = std::sqrt(x_difference * x_difference +
                            y_difference * y_difference);
        }

        /* Create a new instance of the Measurement struct on the first */
        if (0 == s) {
          robots_[id].groundtruth.measurements.push_back(Robot::Measurement(
              robots_[id].synced.measurements[k].time,
              robots_[id].synced.measurements[k].subjects[s], range, bearing));

          /* Move the iterator to the newly created instance*/
          iterator = robots_[id].groundtruth.measurements.end() - 1;
        } else {
          iterator->subjects.push_back(
              robots_[id].synced.measurements[k].subjects[s]);
          iterator->ranges.push_back(range);
          iterator->bearings.push_back(bearing);
        }
      }
    }
  }
}

/**
 * @brief Calculates the relative distance of robots from an ego robot and saves
 * the data.
 * @details The relative distance between robots and the ego robot is calculated
 * using the groundtruth state values extracted from the dataset for each robot.
 * @note This is only for robot 1 at this stage.
 */
void DataHandler::relativeRobotDistance() {
  std::ofstream robot_file;
  std::string filename = data_extraction_directory_ + "Relative_robot.dat";
  robot_file.open(filename);

  if (!robot_file.is_open()) {
    std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
    return;
  }

  robot_file << "# Time [s]	Robot	Ranges [m]	Robot ID\n";
  for (std::size_t k = 0; k < robots_[0].groundtruth.states.size(); k++) {
    for (int id = 0; id < total_robots; id++) {
      double x = robots_[0].groundtruth.states[k].x -
                 robots_[id].groundtruth.states[k].x;
      double y = robots_[0].groundtruth.states[k].y -
                 robots_[id].groundtruth.states[k].y;
      double range = std::sqrt(x * x + y * y);
      robot_file << robots_[0].groundtruth.states[k].time << '\t' << id + 1
                 << '\t' << range << '\t' << 1 << '\n';
    }
  }
  robot_file.close();
}

/**
 * @brief Calculates the relative distance of the landmarks from an ego robot
 * and saves the data.
 * @details The relative distance between the landmarks and the ego robot is
 * calculated using the landmarks and groundtruth state values extracted from
 * the dataset for each landmark and the ego robot respectively.
 * @note This is only for robot 1 at this stage.
 */
void DataHandler::relativeLandmarkDistance() {
  std::ofstream robot_file;
  std::string filename = data_extraction_directory_ + "Relative_landmark.dat";
  robot_file.open(filename);

  if (!robot_file.is_open()) {
    std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
    return;
  }

  robot_file << "# Time [s]	Landmark	Ranges [m]	Robot ID\n";
  for (std::size_t k = 0; k < robots_[0].groundtruth.states.size(); k++) {
    for (int l = 0; l < total_landmarks; l++) {
      double x = robots_[0].groundtruth.states[k].x - landmarks_[l].x;
      double y = robots_[0].groundtruth.states[k].y - landmarks_[l].y;
      double range = std::sqrt(x * x + y * y);
      robot_file << robots_[0].groundtruth.states[k].time << '\t' << l + 6
                 << '\t' << range << '\t' << 1 << '\n';
    }
  }
  robot_file.close();
}

/**
 * @brief Saves all the extracted and processed data in the DataHandler class
 * after data extraction and processing.
 */
void DataHandler::saveExtractedData() {
  auto start = std::chrono::high_resolution_clock::now();

  if (!std::filesystem::exists(data_extraction_directory_)) {
    std::filesystem::create_directories(data_extraction_directory_);
  }

  try {
    double bin_size = 0.001;

    saveStateData();
    saveOdometryData();
    saveMeasurementData();

    saveErrorData();

    saveOdometryErrorPDF(bin_size);
    saveMeasurementErrorPDF(bin_size);

    saveRobotErrorStatistics();

    saveLandmarks();

    // relativeLandmarkDistance();
    // relativeRobotDistance();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "\033[1;32mSaving Extracted Data Complete:\033[0m "
              << data_extraction_directory_ << "\033[0m [" << duration.count()
              << " ms]" << std::endl;
  } catch (std::runtime_error &error) {
    std::cerr << "\033[1;33mUnable to save extracted data\033[0m: "
              << error.what() << std::endl;
    throw;
  }
}

/**
 * @brief Writes the synced (performed by DataHandler::syncData) and raw
 * groundtruth robot state data extracted from the dataset after, which includes
 * its x-coordinate, y-coordinate and heading.
 */
void DataHandler::saveStateData() {

  std::ofstream robot_file;
  std::string filename = data_extraction_directory_ + "Groundtruth-State.dat";

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Unable to create file: " + filename);
  }

  /* Write the file header (# proceeds values for gnuplot to recognise it as a
   * comment) */
  robot_file << "# Time [s]	x [m]	y [m]	orientation [rad]	Raw "
                "(r) / Synced (s)	Robot ID\n";

  /* Loop through the data structures for each robot */
  for (int id = 0; id < total_robots; id++) {
    /* Determine which dataset is larger and set that as the loop iterations
     */
    std::size_t largest_vector_size = std::max(
        {robots_[id].raw.states.size(), robots_[id].groundtruth.states.size()});
    for (std::size_t k = 0; k < largest_vector_size; k++) {
      /* Write the raw ground truth file for the current timestep 'r'  */
      if (k < robots_[id].raw.states.size()) {
        robot_file << robots_[id].raw.states[k].time << '\t'
                   << robots_[id].raw.states[k].x << '\t'
                   << robots_[id].raw.states[k].y << '\t'
                   << robots_[id].raw.states[k].orientation << '\t' << 'r'
                   << '\t' << id + 1 << "\n";
      }

      /* Write the synced ground truth file for the current timestep 'r'  */
      if (k < robots_[id].groundtruth.states.size()) {
        robot_file << robots_[id].groundtruth.states[k].time << '\t'
                   << robots_[id].groundtruth.states[k].x << '\t'
                   << robots_[id].groundtruth.states[k].y << '\t'
                   << robots_[id].groundtruth.states[k].orientation << '\t'
                   << 's' << '\t' << id + 1 << '\n';
      }
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }

  robot_file.close();
}

/**
 * @brief Saves the extracted measurement and calculated groundtruth measurement
 * data from the DataHandler class into .dat files to be plotted by gnuplot.
 * @details Saves both the measurement data (as extracted from the dataset) and
 * the calculated groundtruth measurement values (calculated by
 * DataHandler::calculateGroundtruthMeasurement) into Measurement.dat and
 * Groundtruth-Measurement.dat respectively.
 */
void DataHandler::saveMeasurementData() {

  std::ofstream robot_file;
  std::string filename = data_extraction_directory_ + "Measurement.dat";
  /* If the file already exists, there is no need to rewrite the data again. */
  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("[ERROR]: Could not create file: " + filename);
  }

  /* Write the file header (# proceeds values for gnuplot to recognise it as a
   * comment) */
  robot_file
      << "# Time [s]	Subjects	Ranges [m]	Bearings [m]	"
         "Raw/Synced/Groundtruth	Robot ID	Landmark(l)/Robot(r)\n";

  /* Save the values of the raw and synced measurment values of a given robot
   * into the same file with the last row indicating 'g' for raw  and 'i' for
   * synced.*/
  for (int id = 0; id < total_robots; id++) {

    /* NOTE: when the "raw" measurement data structure is populated, it only
     * adds one element to the members for each time stamp. After
     * interpolation, these values are combined if they have the same time
     * stamp.*/
    for (std::size_t k = 0; k < robots_[id].raw.measurements.size(); k++) {
      /* NOTE: that time stamp grouping is not performed for raw measurements,
       * therefore each subject vector has only one element. */
      int subject_ID = getID(robots_[id].raw.measurements[k].subjects[0]);
      char measurement_type;
      /* Robots from the UTIAS dataset have ID's from [1,5]. */
      if (subject_ID < 6) {
        measurement_type = 'r';
      }
      /* Landmarks from the UTIAS dataset have ID's from [5,20]. */
      else {
        measurement_type = 'l';
      }
      robot_file << robots_[id].raw.measurements[k].time << '\t'
                 << robots_[id].raw.measurements[k].subjects[0] << '\t'
                 << robots_[id].raw.measurements[k].ranges[0] << '\t'
                 << robots_[id].raw.measurements[k].bearings[0] << '\t' << 'r'
                 << '\t' << id + 1 << '\t' << measurement_type << '\n';
    }

    /* Save both the synced and the calculated groundtruth */
    for (std::size_t k = 0; k < robots_[id].synced.measurements.size(); k++) {
      for (std::size_t s = 0;
           s < robots_[id].synced.measurements[k].subjects.size(); s++) {
        int subject_ID =
            getID(robots_[id].groundtruth.measurements[k].subjects[s]);
        char measurement_type;
        if (subject_ID < 6) {
          measurement_type = 'r';
        }
        /* Landmarks from the UTIAS dataset have ID's from [5,20]. */
        else {
          measurement_type = 'l';
        }
        robot_file << robots_[id].synced.measurements[k].time << '\t'
                   << robots_[id].synced.measurements[k].subjects[s] << '\t'
                   << robots_[id].synced.measurements[k].ranges[s] << '\t'
                   << robots_[id].synced.measurements[k].bearings[s] << '\t'
                   << 's' << '\t' << id + 1 << '\t' << measurement_type << '\n';

        robot_file << robots_[id].groundtruth.measurements[k].time << '\t'
                   << robots_[id].groundtruth.measurements[k].subjects[s]
                   << '\t' << robots_[id].groundtruth.measurements[k].ranges[s]
                   << '\t'
                   << robots_[id].groundtruth.measurements[k].bearings[s]
                   << '\t' << 'g' << '\t' << id + 1 << '\t' << measurement_type
                   << '\n';
      }
    }

    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }
  robot_file.close();
}

/**
 * @brief Saves the measured odometry data from the DataHandler class into a
 * .dat file to be plotted by gnuplot.
 * @details Saves the odometry data (as extracted from the dataset) into
 * Odometry.dat.
 */
void DataHandler::saveOdometryData() {

  std::ofstream robot_file;
  std::string filename = data_extraction_directory_ + "Odometry.dat";
  /* If the file already exists, there is no need to rewrite the data again. */

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Unable to create file: " + filename);
  }

  /* Write the file header (# proceeds values for gnuplot to recognise it as a
   * comment) */
  robot_file
      << "# Time [s]	Forward Velocity [m/s]	Angular Velocity "
         "[rad/s]	Raw (r)/Synced(s)/Groundtruth(g)	Robot ID\n";

  for (int id = 0; id < total_robots; id++) {
    std::size_t largest_vector_size = std::max(
        {robots_[id].raw.odometry.size(), robots_[id].synced.odometry.size()});

    for (std::size_t k = 0; k < largest_vector_size; k++) {
      if (k < robots_[id].raw.odometry.size()) {
        robot_file << robots_[id].raw.odometry[k].time << '\t'
                   << robots_[id].raw.odometry[k].forward_velocity << '\t'
                   << robots_[id].raw.odometry[k].angular_velocity << '\t'
                   << 'r' << '\t' << id + 1 << '\n';
      }

      if (k < robots_[id].synced.odometry.size()) {
        robot_file << robots_[id].synced.odometry[k].time << '\t'
                   << robots_[id].synced.odometry[k].forward_velocity << '\t'
                   << robots_[id].synced.odometry[k].angular_velocity << '\t'
                   << 's' << '\t' << id + 1 << '\n';

        robot_file << robots_[id].groundtruth.odometry[k].time << '\t'
                   << robots_[id].groundtruth.odometry[k].forward_velocity
                   << '\t'
                   << robots_[id].groundtruth.odometry[k].angular_velocity
                   << '\t' << 'g' << '\t' << id + 1 << '\n';
      }
    }

    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }

  robot_file.close();
}

/**
 * @brief Saves calculated error between the measured data (as extracted form
 * the dataset) and the calculated groundtruth values.
 * @details The error between the measured odometry and the calculated
 * groundtruth odometry produced by Robot::calculateOdometryError and the error
 * between the measured measurements and the groundtruth measurements produced
 * bye Robot::calculateMeasurementError is saved into their respective .dat
 * files.
 */
void DataHandler::saveErrorData() {

  std::ofstream robot_file;
  std::string filename = data_extraction_directory_ + "Odometry-Error.dat";
  /* If the file already exists, there is no need to rewrite the data again. */

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Could not create file: " + filename);
  }

  /* Write the file header (# proceeds values for gnuplot to recognise it as a
   * comment) */
  robot_file << "# Time [s]	Forward Velocity [m/s]	Angular Velocity "
                "[rad/s]	Robot ID\n";

  /* Save the error values of the odometry.*/
  for (int id = 0; id < total_robots; id++) {
    for (std::size_t k = 0; k < robots_[id].error.odometry.size(); k++) {
      robot_file << robots_[id].error.odometry[k].time << '\t'
                 << robots_[id].error.odometry[k].forward_velocity << '\t'
                 << robots_[id].error.odometry[k].angular_velocity << '\t'
                 << id + 1 << '\n';
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }
  robot_file.close();

  filename = data_extraction_directory_ + "Measurement-Error.dat";

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Unable to create file: " + filename);
  }

  /* Write the file header (# proceeds values for gnuplot to recognise it as a
   * comment) */
  robot_file
      << "# Time [s]	Subject	Range [m]	Bearing[rad]	Robot ID\n";

  /* Save the error values of the odometry.*/
  for (int id = 0; id < total_robots; id++) {

    for (std::size_t k = 0; k < robots_[id].error.measurements.size(); k++) {
      for (std::size_t s = 0;
           s < robots_[id].error.measurements[k].subjects.size(); s++) {
        robot_file << robots_[id].error.measurements[k].time << '\t'
                   << robots_[id].error.measurements[k].subjects[s] << '\t'
                   << robots_[id].error.measurements[k].ranges[s] << '\t'
                   << robots_[id].error.measurements[k].bearings[s] << '\t'
                   << id + 1 << '\n';
      }
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }
  robot_file.close();
}

/**
 * @brief Performs binning on the odometry error for the determination of a
 * discretized Probability Density Function (PDF).
 * succesfull.
 * @param[in] bin_size the size of the bins (denoting the range of values) that
 * odometry measurement values gets grouped into.
 * @note The bin count is actually the area contribution of the odometry error
 * for a given odometry measurement. This means that the output is a discretized
 * pdf, where the sum of the area of all the bins should equal 1. This is done
 * for better visualisation when fitting a Gaussian curve to the data.
 */
void DataHandler::saveOdometryErrorPDF(double bin_size) {
  std::ofstream robot_file;

  /* Forward Velocity */
  std::string filename =
      data_extraction_directory_ + "Forward-Velocity-Error-PDF.dat";

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Could not create file: " + filename);
  }

  /* Write the file header (# proceeds values for gnuplot to recognise it as a
   * comment) */
  robot_file << "# Bin Centre	Bin Width	Bin Count	Robot ID\n";

  /* Save the plot data for the Forward Velocity Error  */
  for (int id = 0; id < total_robots; id++) {
    std::unordered_map<int, double> forward_velocity_bin_counts;

    for (const auto &odometry : robots_[id].error.odometry) {
      int bin_index =
          static_cast<int>(std::floor(odometry.forward_velocity / bin_size));
      /* NOTE: The bin count is actually the area contribution of the odometry
       * error for the given measurement. This means that the output is a
       * discretized pdf, where the sum of the area of all the bins should
       * equal 1. This is done for better visualisation when fitting a
       * Gaussian curve to the data. */
      forward_velocity_bin_counts[bin_index] +=
          1.0 / (robots_[id].error.odometry.size() * bin_size);
    }

    for (const auto &[bin_index, count] : forward_velocity_bin_counts) {
      double bin_start = bin_index * bin_size;
      double bin_end = bin_start + bin_size;

      robot_file << (bin_start + bin_end) / 2 << '\t' << bin_size << "\t"
                 << count << '\t' << id + 1 << '\n';
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }

  robot_file.close();

  /* Angular velocity */
  filename = data_extraction_directory_ + "Angular-Velocity-Error-PDF.dat";

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Could not create file: " + filename);
  }

  robot_file << "# Bin Centre	Bin Width	Count	Robot ID\n";

  for (int id = 0; id < total_robots; id++) {
    /* Save the plot data for the Angular Velocity Error  */

    std::unordered_map<int, double> angular_velocity_bin_counts;

    for (const auto &odometry : robots_[id].error.odometry) {
      int bin_index =
          static_cast<int>(std::floor(odometry.angular_velocity / bin_size));
      angular_velocity_bin_counts[bin_index] +=
          1.0 / (robots_[id].error.odometry.size() * bin_size);
    }

    for (const auto &[bin_index, count] : angular_velocity_bin_counts) {
      double bin_start = bin_index * bin_size;
      double bin_end = bin_start + bin_size;

      robot_file << (bin_start + bin_end) / 2 << '\t' << bin_size << "\t"
                 << count << '\t' << id + 1 << '\n';
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }
  robot_file.close();
}

/**
 * @brief Performs binning on the measurement error for the determination of a
 * discretized Probability Density Function (PDF).
 * @param[in] bin_size the size of the bins (denoting the range of values) that
 * measurement values gets grouped into.
 * @note The bin count is actually the area contribution of the odometry error
 * for a given odometry measurement. This means that the output is a discretized
 * pdf, where the sum of the area of all the bins should equal 1. This is done
 * for better visualisation when fitting a Gaussian curve to the data.
 */
void DataHandler::saveMeasurementErrorPDF(double bin_size) {
  std::ofstream robot_file;

  /* Range */
  std::string filename =
      this->data_extraction_directory_ + "Range-Error-PDF.dat";

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Could not create file: " + filename);
  }

  robot_file << "# Bin Centre	Bin Width	Bin Count	Robot ID\n";
  /* Save the plot data for the Forward Velocity Error  */
  for (int id = 0; id < total_robots; id++) {

    double number_of_measurements = 0.0;
    for (std::size_t k = 0; k < robots_[id].error.measurements.size(); k++) {
      number_of_measurements += robots_[id].error.measurements[k].ranges.size();
    }

    std::unordered_map<int, double> range_bin_counts;
    for (const auto &measurement : robots_[id].error.measurements) {
      for (auto range : measurement.ranges) {
        int bin_index = static_cast<int>(std::floor(range / bin_size));
        range_bin_counts[bin_index] +=
            1.0 / (number_of_measurements * bin_size);
      }
    }

    for (const auto &[bin_index, count] : range_bin_counts) {
      double bin_start = bin_index * bin_size;
      double bin_end = bin_start + bin_size;

      robot_file << (bin_start + bin_end) / 2 << '\t' << bin_size << "\t"
                 << count << '\t' << id + 1 << '\n';
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }

  robot_file.close();

  /* Bearing */
  filename = data_extraction_directory_ + "Bearing-Error-PDF.dat";

  robot_file.open(filename);

  if (!robot_file.is_open()) {
    throw std::runtime_error("Could not create file: " + filename);
  }

  robot_file << "# Bin Centre	Bin Width	Count	Robot ID\n";

  for (int id = 0; id < total_robots; id++) {
    /* Save the plot data for the Angular Velocity Error  */

    double number_of_measurements = 0.0;

    for (std::size_t k = 0; k < robots_[id].error.measurements.size(); k++) {
      number_of_measurements += robots_[id].error.measurements[k].ranges.size();
    }

    std::unordered_map<int, double> bearing_bin_counts;

    for (const auto &measurement : robots_[id].error.measurements) {
      for (auto bearing : measurement.bearings) {
        int bin_index = static_cast<int>(std::floor(bearing / bin_size));
        bearing_bin_counts[bin_index] +=
            1.0 / (number_of_measurements * bin_size);
      }
    }

    for (const auto &[bin_index, count] : bearing_bin_counts) {
      double bin_start = bin_index * bin_size;
      double bin_end = bin_start + bin_size;

      robot_file << (bin_start + bin_end) / 2 << '\t' << bin_size << "\t"
                 << count << '\t' << id + 1 << '\n';
    }
    /* Add two empty lines after robot entires for gnuplot */
    robot_file << '\n';
    robot_file << '\n';
  }
  robot_file.close();
}

/**
 * @brief Saves the sample mean and sample variance of the measured odometry and
 * tracking data for each robot.
 */
void DataHandler::saveRobotErrorStatistics() {
  std::string filename =
      this->data_extraction_directory_ + "/Robot-Error-Statistics.dat";

  std::ofstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error(" Unable to create file:  " + filename);
  }

  /* Write file header. */
  file << "# Robot ID	Forward Velocity Mean [m]	Forward Velocity "
          "Variance [m^2]	Angular Velocity Mean [rad]	Angular "
          "Veolcity [rad^2]	Range Mean [m]	Range Variance [m^2]	"
          "Bearing Mean [rad]	Bearing Variance [rad^2]\n";

  for (unsigned short int id = 0; id < total_robots; id++) {
    file << id + 1 << '\t' << robots_[id].forward_velocity_error.mean << '\t'
         << robots_[id].forward_velocity_error.variance << '\t'
         << robots_[id].angular_velocity_error.mean << '\t'
         << robots_[id].angular_velocity_error.variance << '\t'
         << robots_[id].range_error.mean << '\t'
         << robots_[id].range_error.variance << '\t'
         << robots_[id].bearing_error.mean << '\t'
         << robots_[id].bearing_error.variance << '\n';

    /* Two blank line for gnuplot to be able to automatically seperate data
     * from different robots */
    file << '\n';
    file << '\n';
  }

  file.close();
}

void DataHandler::saveLandmarks() {
  std::string filename = data_extraction_directory_ + "/landmarks.dat";

  std::ofstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Unable to create file: " + filename);
  }

  file << "# ID	Barcode	x-coordinate [m]	y-coordinate "
          "[m]	x std-dev [m]	y std-dev [m]\n";
  for (unsigned short int id = 0; id < total_landmarks; id++) {
    file << landmarks_[id].id << '\t' << landmarks_[id].barcode << '\t'
         << landmarks_[id].x << '\t' << landmarks_[id].y << '\t'
         << landmarks_[id].x_std_dev << '\t' << landmarks_[id].y_std_dev
         << '\n';
  }

  file.close();
}

/**
 * @brief Saves the error and absolute error between estimated state and the
 * groudtruth state.
 */
void DataHandler::saveStateError() {
  if (!std::filesystem::exists(data_inference_directory)) {
    std::filesystem::create_directories(data_inference_directory);
  }

  std::string filename = data_inference_directory + "/state_error.dat";

  std::ofstream file(filename);

  file << "#Time [s]  x Error [m] y error [m] orienation error [rad]  Robot "
          "ID\n";

  for (unsigned short id = 0; id < total_robots; id++) {
    /* Populate the error state if it has not yet been done. */
    if (robots_[id].error.states.empty()) {
      robots_[id].calculateStateError();
    }
    if (total_synced_datapoints > robots_[id].error.states.size()) {
      throw std::runtime_error("Robot " + std::to_string(id) +
                               " has less synced datapoints than groundtruth "
                               "points. Check your filter implementation.");
    }

    for (unsigned long k = 0; k < total_synced_datapoints; k++) {
      file << robots_[id].error.states[k].time << '\t'
           << robots_[id].error.states[k].x << '\t'
           << robots_[id].error.states[k].y << '\t'
           << robots_[id].error.states[k].orientation << '\t' << robots_[id].id
           << '\n';
    }

    file << '\n';
    file << '\n';
  }
}

/**
 * @brief Getter for the array of Barcodes.
 * @return a reference the barcodes integer vector extracted from the barcodes
 * data file: Barcodes.dat.
 * @note if the dataset has not been set, the function will throw a
 * std::runtime_error.
 */
std::vector<unsigned short int> &DataHandler::getBarcodes() {
  if ("" == this->dataset_) {
    throw std::runtime_error(
        "Dataset has not been specified during object instantiation. Please "
        "ensure you call void setDataSet(std::string) before attempting to get "
        "data.");
  }
  return barcodes_;
}

/**
 * @brief Create the directory for the state plots.
 */
void DataHandler::createStatePlotDirectory() {

  std::string plots_directory = data_extraction_directory_ + "plots/";

  /* Check if the data extraction directory exists */
  if (!std::filesystem::exists(data_extraction_directory_)) {
    saveExtractedData();
  }
  /* Create the plots directory (if it doesn't exist) */
  if (!std::filesystem::exists(plots_directory)) {
    if (!std::filesystem::create_directory(plots_directory)) {
      throw std::runtime_error("Failed to create directory: " +
                               plots_directory);
    }
  }
  /* Create the directory for the state plots. */
  std::string state_directory = plots_directory + "State";
  if (!std::filesystem::exists(state_directory)) {
    if (!std::filesystem::create_directory(state_directory)) {
      throw("Failed to create directory: " + state_directory);
    }
  }
}

/**
 * @brief Checks Create the directories required for the measurement plots.
 */
void DataHandler::createMeasurementPlotDirectories() {
  std::string plots_directory = data_extraction_directory_ + "plots/";

  /* Check if the data extraction directory exists */
  if (!std::filesystem::exists(data_extraction_directory_)) {
    saveExtractedData();
  }
  /* Create the plots directory (if it doesn't exist) */
  if (!std::filesystem::exists(plots_directory)) {
    if (!std::filesystem::create_directory(plots_directory)) {
      throw std::runtime_error("Failed to create directory: " +
                               plots_directory);
    }
  }
  /* Create the Range Error sub-directory (if it doesn't exist) */
  std::string range_directory = plots_directory + "Range";
  if (!std::filesystem::exists(range_directory)) {
    if (!std::filesystem::create_directory(range_directory)) {
      throw std::runtime_error("Failed to create directory: " +
                               range_directory);
    }
  }

  /* Create the Bearing Error subdirectory (if it doesn't exist) */
  std::string bearing_directory = plots_directory + "Bearing";
  if (!std::filesystem::exists(bearing_directory)) {
    if (!std::filesystem::create_directory(bearing_directory)) {
      throw std::runtime_error("Failed to create directory: " +
                               bearing_directory);
    }
  }

  /* Create the Forward-Velocity Error subdirectory (if it doesn't exist) */
  std::string forward_velocity_directory = plots_directory + "Forward-Velocity";
  if (!std::filesystem::exists(forward_velocity_directory)) {
    if (!std::filesystem::create_directory(forward_velocity_directory)) {
      throw std::runtime_error("Failed to create directory: " +
                               forward_velocity_directory);
    }
  }

  /* Create the Forward-Velocity Error subdirectory (if it doesn't exist) */
  std::string angular_velocity_directory = plots_directory + "Angular-Velocity";
  if (!std::filesystem::exists(angular_velocity_directory)) {
    if (!std::filesystem::create_directory(angular_velocity_directory)) {
      throw std::runtime_error("Failed to create directory: " +
                               angular_velocity_directory);
    }
  }
}

/**
 * @brief Runs the GNUplot scripts to save plots related to the extracted data.
 * @param[in] file_type The file type of the output plot.
 * @note At this stage only png and pdf file types are implemented. Any other
 * file type will throw a std::runtime_error.
 */
void DataHandler::plotExtractedData(std::string file_type) {
  /* Start the Timer */
  auto start = std::chrono::high_resolution_clock::now();
  std::string plots_directory = data_extraction_directory_ + "plots/";

  /* Create the directories required for all the plots. */
  createStatePlotDirectory();
  createMeasurementPlotDirectories();

  plotPDFs(file_type);
  plotMeasurements(file_type);
  plotError(file_type);
  plotStates(file_type);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  std::cout << "\033[1;32mExtracted Data Plotting Complete:\033[0m \033[3m"
            << plots_directory << "\033[0m [" << duration.count() << " ms]"
            << std::endl;
}

/**
 * @brief Plot the error PDF's of the odometry measurements (forward and angular
 * velocity) along with the range and bearing error PDF.
 * @param[in] file_type The file type of the output plot.
 */
void DataHandler::plotPDFs(std::string file_type) {
  createMeasurementPlotDirectories();

  std::string plots_directory = data_extraction_directory_ + "plots/";
  /* Execute gnuplot command */
  std::string gnuplot_script_path =
      std::string(LIB_DIR) + "/scripts/measurement-error-pdf.gp";

  std::string command = "gnuplot -e \"dataset_directory='" +
                        data_extraction_directory_ + "'; plots_directory='" +
                        plots_directory + "'; file_type='" + file_type +
                        "'\" " + gnuplot_script_path;

  int ret = system(command.c_str());

  if (ret != 0) {
    throw std::runtime_error(
        "Unable to plot measuremet error PDF. Gnuplot failed with code: " +
        std::to_string(ret));
  }
}

/**
 * @brief Plot the measurment error odometry (forward and angular velocity)
 * along with the range and bearing error.
 * @param[in] file_type The file type of the output plot.
 */
void DataHandler::plotError(std::string file_type) {
  createMeasurementPlotDirectories();
  std::string plots_directory = data_extraction_directory_ + "plots/";

  std::string gnuplot_script_path =
      std::string(LIB_DIR) + "/scripts/measurement-error.gp";
  std::string command = "gnuplot -e \"dataset_directory='" +
                        data_extraction_directory_ + "'; plots_directory='" +
                        plots_directory + "'; file_type='" + file_type +
                        "'\" " + gnuplot_script_path;

  int ret = system(command.c_str());

  if (ret != 0) {
    throw std::runtime_error(
        "Unable to plot measurement error. Gnuplot failed with code: " +
        std::to_string(ret));
  }
}

/**
 * @brief Plot the raw, synced and groundtruth odometry measurements, along with
 * the raw, synced and groundtruth range and bearing measurements.
 * @param[in] file_type The file type of the output plot.
 */
void DataHandler::plotMeasurements(std::string file_type) {
  createMeasurementPlotDirectories();
  std::string plots_directory = data_extraction_directory_ + "plots/";

  std::string gnuplot_script_path =
      std::string(LIB_DIR) + "/scripts/measurement-dataset.gp";
  std::string command = "gnuplot -e \"dataset_directory='" +
                        data_extraction_directory_ + "'; plots_directory='" +
                        plots_directory + "'; file_type='" + file_type +
                        "'\" " + gnuplot_script_path;
  int ret = system(command.c_str());

  if (ret != 0) {
    throw std::runtime_error(
        "Unable to plot measurements. Gnuplot failed with code: " +
        std::to_string(ret));
  }
}

/**
 * @brief plot the robots states, which includes its x,y and orientation, along
 * with a x,y position plot.
 * @param[in] file_type The file type of the output plot.
 */
void DataHandler::plotStates(std::string file_type) {
  createStatePlotDirectory();

  std::string plots_directory = data_extraction_directory_ + "plots/";
  std::string gnuplot_script_path =
      std::string(LIB_DIR) + "/scripts/groundtruth-dataset.gp";
  std::string command = "gnuplot -e \"dataset_directory='" +
                        data_extraction_directory_ + "'; plots_directory='" +
                        plots_directory + "'; file_type='" + file_type +
                        "'\" " + gnuplot_script_path;
  int ret = system(command.c_str());

  if (ret != 0) {
    throw std::runtime_error("Gnuplot failed with code: " +
                             std::to_string(ret));
  }
}

/**
 * @brief Plots the data generated by the cooperative positioning filter
 * corresponding to the estimated system state.
 * @param[in] file_type The file type of the output plot.
 */
void DataHandler::plotInferenceError(std::string file_type) {

  auto start = std::chrono::high_resolution_clock::now();

  std::string plots_directory = data_inference_directory + "/plots/";

  if (!std::filesystem::exists(plots_directory)) {
    std::filesystem::create_directory(plots_directory);
  }

  std::string gnuplot_script_path =
      std::string(LIB_DIR) + "/scripts/state_error.gp";

  std::string command = "gnuplot -e \"dataset_directory='" +
                        data_inference_directory + "'; plots_directory='" +
                        plots_directory + "'; file_type='" + file_type +
                        "'\" " + gnuplot_script_path;
  int ret = system(command.c_str());

  if (ret != 0) {
    throw std::runtime_error("Gnuplot failed with code: " +
                             std::to_string(ret));
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  std::cout << "\033[1;32mInference Data Plotting Complete:\033[0m \033[3m"
            << plots_directory << "\033[0m [" << duration.count() << " ms]"
            << std::endl;
}

/**
 * @brief Searches trough the list of barcodes to find the index ID of the
 * robot or landmark.
 * @param[in] barcode the barcode value for which the ID needs to be found.
 * @return the ID of the robot of landmark. If the ID is not found -1 is
 * returned.
 * @note the ID is one larger than it's index. Therefore, robot 4 has ID 4 and
 * index 3 in the array DataHandler::robots_.
 * @note if the dataset has not been set, the function will throw a
 * std::runtime_error.
 */
int DataHandler::getID(unsigned short int barcode) {
  for (int i = 0; i < total_barcodes; i++) {
    if (barcodes_[i] == barcode) {
      return (i + 1);
    }
  }
  return -1;
}

/**
 * @brief Getter for the array of Landmarks.
 * @return a reference to the Landmarks class vector, populated by extracting
 * data form Landmarks.dat.
 */
std::vector<Landmark> &DataHandler::getLandmarks() {
  if ("" == this->dataset_) {
    throw std::runtime_error(
        "Dataset has not been specified during object instantiation. Please "
        "ensure you call void setDataSet(std::string) before attempting to get "
        "data.");
  }
  return landmarks_;
}

/**
 * @brief Getter for the array of robots.
 * @return a reference to the Robot class vector, populated by extracting
 * datefrom Robotx_Groundtruth.dat, Robotx_Odometry.dat, and
 * Robotx_Measurement.dat.
 * @note if the dataset has not been set, the function will throw a
 * std::runtime_error.
 */
std::vector<Robot> &DataHandler::getRobots() {
  if ("" == this->dataset_) {
    throw std::runtime_error(
        "Dataset has not been specified during object instantiation. Please "
        "ensure you call void setDataSet(std::string) before attempting to get "
        "data.");
  }

  return robots_;
}

/**
 * @brief Getter for the DataHandler::sampling_period_ field.
 * @return the sampling period set by the user.
 * @note DataExtractor::sampling_period_ has a default value of 0.02.
 */
double DataHandler::getSamplePeriod() { return sampling_period_; }

/**
 * @brief Getter for the DataHandler::total_robots field.
 * @return the number of robots set by the user dataset.
 * @note the field is initialised to zero, therefore if it is not set, a
 * std::runtime_error will be throw.
 */
unsigned short int DataHandler::getNumberOfRobots() {
  if (0 == total_robots) {
    throw std::runtime_error("The total number of robots have not been set.");
  }
  return total_robots;
}

/**
 * @brief Getter for the DataHandler::total_landmarks field.
 * @return the number of landmarks set by the user or the dataset.
 * @note the field is initialised to zero, therefore if it is not set, a
 * std::runtime_error will be throw.
 */
unsigned short int DataHandler::getNumberOfLandmarks() {
  if (0 == total_landmarks) {
    throw std::runtime_error(
        "The total number of landmarks have not been set.");
  }
  return total_landmarks;
}

/**
 * @brief Getter for the DataHandler::total_barcodes field.
 * @return the number of landmarks set by the user or the dataset.
 * @note the field is initialised to zero, therefore if it is not set, a
 * std::runtime_error will be throw.
 */
unsigned short int DataHandler::getNumberOfBarcodes() {
  if (0 == total_barcodes) {
    throw std::runtime_error("The total number of barcodes have not been set.");
  }
  return total_barcodes;
}

/**
 * @brief Getter for the DataHandler::getNumberOfSyncedDatapoints field.
 */
unsigned long DataHandler::getNumberOfSyncedDatapoints() {
  return total_synced_datapoints;
}
