/**
 * @file DataHandler.h
 * @brief Header file of the Handler class,
 * @author Daniel Ingham
 * @date 2025-04-04
 */
#ifndef INCLUDE_INCLUDE_DATA_HANDLER_H_
#define INCLUDE_INCLUDE_DATA_HANDLER_H_

#include <cmath>   // std::floor
#include <cstdlib> // system
#include <string>  // std::string
#include <vector>  // std::vector

#include "Landmark.h"
#include "Robot.h"
#include "Simulator.h"

namespace Data {

/**
 * @struct DataHandlerDefaults
 * Contains the default parameters for the Handler class constructor so the
 * the argument handler can set the defaults if the user does not provide input
 * for those arguments.
 */
struct HandlerDefaults {
  /**
   * Time betweenn samples. Use by the linear interpolation to sync the data
   * timesteps.
   */
  static constexpr double kSamplePeriod = 0.02;

  /**
   * The subdirectory within the "output" directory where the processed data
   * is stored.
   */
  static const inline std::string kOutputDir = "";
};

/**
 * @class Handler
 * @brief Extracts the data from the UTIAS Multi-robot Localisaion and Mapping
 * Dataset.
 * @details The class extracts the textfile dataset form UTIAS multi-robot
 * localisation and mapping dataset into three members:
 * Data::Handler::barcodes_, Data::Handler::landmarks_, and
 * Data::Handler::robots_.
 */
class Handler {
public:
  /* Constructors */
  Handler();

  Handler(const std::string &,
          const std::string &output_directory = HandlerDefaults::kOutputDir,
          const double &sampling_period = HandlerDefaults::kSamplePeriod);

  Handler(
      const unsigned long int,
      double sample_period = SimulationDefaults::kSamplePeriod,
      const unsigned short number_of_robots = SimulationDefaults::kRobots,
      const unsigned short number_of_landmarks = SimulationDefaults::kLandmarks,
      const std::string &output_directory = SimulationDefaults::kOutputDir);

  /* Setters */
  void
  setDataSet(const std::string &,
             const std::string &output_directory = HandlerDefaults::kOutputDir,
             const double &sampling_period = HandlerDefaults::kSamplePeriod);

  void setSimulation(
      unsigned long int,
      const unsigned short number_of_robots = SimulationDefaults::kRobots,
      const unsigned short number_of_landmarks = SimulationDefaults::kLandmarks,
      double sample_period = SimulationDefaults::kSamplePeriod,
      const std::string &output_directory = SimulationDefaults::kOutputDir,
      const unsigned long seed = SimulationDefaults::kSeed);

  std::string getDatasetName();
  std::string getDataExtractionDirectory();
  std::string getDataInferenceDirectory();

  std::vector<Robot> &getRobots();
  const std::vector<Landmark> &getLandmarks() const;
  const std::vector<unsigned short int> &getBarcodes() const;

  const double getSamplePeriod() const;

  const unsigned short getNumberOfRobots() const;
  const unsigned short getNumberOfLandmarks() const;
  const unsigned short getNumberOfBarcodes() const;
  const unsigned long getNumberOfSyncedDatapoints() const;
  const std::vector<size_t> getNumberOfSyncedMeasurements() const;

  const int getID(const unsigned short int) const;

  /* Output of Extracted Data */
  void saveExtractedData();
  void saveInferenceData();

  void plotExtractedData(std::string file_type = "");
  void plotPDFs(std::string file_type = "");
  void plotError(std::string file_type = "");
  void plotMeasurements(std::string file_type = "");
  void plotStates(std::string file_type = "");

  void plotInferenceData(std::string file_type = "");

private:
  /**
   * @brief Folder location for the dataset.
   */
  std::string dataset_ = "";

  /**
   * @brief Folder location of the output directory inside the "./output/"
   * folder.
   */
  std::string output_directory_ = "";

  /**
   * @brief Folder location for the output data correspoding the the data
   * extraction process.
   */
  std::string data_extraction_directory_ = "";

  /**
   * @brief Folder location for the output data corresponding to the infernce
   * peformance.
   */
  std::string data_inference_directory_ = "";

  /**
   * @brief the desired sample period for resampling the data to sync the
   * timesteps between the vehicles.
   */
  double sampling_period_ = 0.2;

  /**
   * @brief The total number of landmarks in the dataset.
   */
  unsigned short total_landmarks_ = 0U;

  /**
   * @brief The total number of robots in the dataset.
   */
  unsigned short total_robots_ = 0U;

  /**
   * @brief the total number of barcodes in the dataset.
   * @note the value of this variable is the summation of the
   * Data::Handler::TOTAL_LANDMARKS and Data::Handler::TOTAL_ROBOTS.
   */
  unsigned short total_barcodes_ = 0U;

  /**
   * @brief The number of datapoints after calling Robot::sync_data.
   */
  unsigned long total_synced_datapoints_ = 0U;

  /**
   * @brief the total number of synced measurements for each robot.
   */
  std::vector<size_t> total_synced_measurements_;

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
   * using Data::Handler::calculateGroundtruthOdometry and
   * Data::Handler::calculateGroundtruthMeasurement.
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
   * be used as a check by Data::Handler::readLandmarks to see if all the
   * barcodes were correctly set.
   */
  std::vector<unsigned short int> barcodes_;

  /**
   * @brief Simulator class responsible for creating odometry, and measurement
   * data for the robots, and assigning positions to the landmarks.
   */
  Simulator simulator_;

  void setOutputDirectory(const std::string &, const std::string &);

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

  void setNumberOfSyncedMeasurements();

  void createStatePlotDirectory();
  void createMeasurementPlotDirectories();

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
};
} // namespace Data
#endif // INCLUDE_INCLUDE_DATA_EXTRACTOR_H_
