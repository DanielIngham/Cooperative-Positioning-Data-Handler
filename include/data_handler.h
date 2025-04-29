/**
 * @file data_extractor.h
 * @brief Header file of the DataHandler class,
 * @author Daniel Ingham
 * @date 2025-04-04
 */
#ifndef INCLUDE_INCLUDE_DATA_HANDLER_H_
#define INCLUDE_INCLUDE_DATA_HANDLER_H_

#include <algorithm>     // std::remove_if and std::find
#include <chrono>        // std::chrono
#include <cmath>         // std::floor
#include <cstdlib>       // system
#include <filesystem>    // std::filesystem
#include <format>        // std::format
#include <fstream>       // std::ifstream
#include <iostream>      // std::cout
#include <sstream>       // std::ostringstream
#include <stdexcept>     // std::runtime_error
#include <string>        // std::string
#include <unordered_map> // std::unordered_map
#include <vector>        // std::vector

#include "landmark.h"
#include "robot.h"
#include "simulator.h"

/**
 * @class DataHandler
 * @brief Extracts the data from the UTIAS Multi-robot Localisaion and Mapping
 * Dataset.
 * @details The class extracts the textfile dataset form UTIAS multi-robot
 * localisation and mapping dataset into three members: DataHandler::barcodes_,
 * DataHandler::landmarks_, and DataHandler::robots_.
 */
class DataHandler {
public:
  /* Constructors */
  DataHandler();
  explicit DataHandler(const std::string &,
                       const double &sampling_period = 0.02);
  DataHandler(const unsigned long int, double, const unsigned short,
              const unsigned short);

  /* Setters */
  void setDataSet(const std::string &, const double &sampling_period = 0.02);
  void setSimulation(unsigned long int, double, const unsigned short,
                     const unsigned short);

  /* Getters */
  std::vector<Landmark> &getLandmarks();
  std::vector<Robot> &getRobots();
  std::vector<unsigned short int> &getBarcodes();

  double getSamplePeriod();

  unsigned short int getNumberOfRobots();
  unsigned short int getNumberOfLandmarks();
  unsigned short int getNumberOfBarcodes();

  int getID(unsigned short int);

  /* Output of Extracted Data */
  void saveExtractedData();

  /* Save Dataset Extraction Data */
  void saveStateData();
  void saveOdometryData();
  void saveMeasurementData();
  void saveErrorData();

  void saveOdometryErrorPDF(double);
  void saveMeasurementErrorPDF(double);

  void saveRobotErrorStatistics();
  void saveLandmarks();

  void relativeRobotDistance();
  void relativeLandmarkDistance();

  void plotExtractedData();
  void plotPDFs();
  void plotError();
  void plotMeasurements();
  void plotStates();

private:
  /**
   * @brief Folder location for the dataset.
   */
  std::string dataset_ = "";

  /**
   * @brief Folder location for the output data.
   */
  std::string data_extraction_directory_ = "";

  /**
   * @brief the desired sample period for resampling the data to sync the
   * timesteps between the vehicles.
   */
  double sampling_period_ = 0.2;

  /**
   * @brief The total number of landmarks in the dataset.
   */
  unsigned short int total_landmarks = 0;

  /**
   * @brief The total number of robots in the dataset.
   */
  unsigned short int total_robots = 0;

  /**
   * @brief the total number of barcodes in the dataset.
   * @note the value of this variable is the summation of the
   * DataHandler::TOTAL_LANDMARKS and DataHandler::TOTAL_ROBOTS.
   */
  unsigned short int total_barcodes = 0;

  /**
   * @brief All landmarks containing all the data extracted form
   * "Landmarks.dat".
   */
  std::vector<Landmark> landmarks_;

  /**
   * @brief std::vector the Robot class. This class contains all information
   * pertaining to the robots. This class contains all information pertaining to
   * the robots. The 'Robotx_Odometry.dat', 'Robotx_Measurement.dat' and
   * 'Robotx_Groundtruth.dat' are used to populate the class. Additionally, the
   * groundtruth values that are not provided by the dataset are calculated
   * using DataHandler::calculateGroundtruthOdometry and
   * DataHandler::calculateGroundtruthMeasurement.
   */
  std::vector<Robot> robots_;

  /**
   * @brief  List of all barcodes corresponding to the robots and landmarks.
   * @note The index of the element in the array corresponds to its ID minus
   * one.
   * @details The list of barcodes corresponding to both the robots and the
   * landmarks exctracted from the 'Barcodes.dat' file. The UTIAS dataset
   * contains 20 barcodes: 5 robots and 15 landmarks. All barcodes are
   * initialised to 0. Since none of the barcodes have a value of 0, this will
   * be used as a check by DataHandler::readLandmarks to see if all the barcodes
   * were correctly set.
   */
  std::vector<unsigned short int> barcodes_;

  /**
   * @brief Simulator class responsible for creating odometry, and measurement
   * data for the robots, and assigning positions to the landmarks.
   */
  Simulator simulator;

  /* Extracting Data from the Dataset */
  void readBarcodes(const std::string &);
  void readLandmarks(const std::string &);
  void readGroundTruth(const std::string &, int);
  void readOdometry(const std::string &, int);
  void readMeasurements(const std::string &, int);

  /* Processing the Data for Filtering */
  void syncData(const double &);

  void calculateGroundtruthOdometry();
  void calculateGroundtruthMeasurement();

  void createStatePlotDirectory();
  void createMeasurementPlotDirectories();
};

#endif // INCLUDE_INCLUDE_DATA_EXTRACTOR_H_
