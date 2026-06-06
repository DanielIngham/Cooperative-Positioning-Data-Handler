/**
 * @file DataHandler.h
 * @brief Header file of the Handler class,
 * @author Daniel Ingham
 * @date 2025-04-04
 */
#ifndef INCLUDE_INCLUDE_DATA_HANDLER_H_
#define INCLUDE_INCLUDE_DATA_HANDLER_H_

#include <cmath> // std::floor
#include <cstddef>
#include <cstdlib> // system
#include <map>
#include <string> // std::string
#include <vector> // std::vector

#include "UtiasMrclam/Simulator.hpp"
#include "UtiasMrclam/agents/Landmark.hpp"
#include "UtiasMrclam/agents/Robot.hpp"

namespace utias::mrclam {

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
  static constexpr double kSamplePeriod{0.02};

  /**
   * The subdirectory within the "output" directory where the processed data
   * is stored.
   */
  static const inline std::string kOutputDir{};
};

static constexpr const char *Set[]{
    "MRCLAM_Dataset1", "MRCLAM_Dataset2", "MRCLAM_Dataset3",
    "MRCLAM_Dataset4", "MRCLAM_Dataset5", "MRCLAM_Dataset6",
    "MRCLAM_Dataset7", "MRCLAM_Dataset8", "MRCLAM_Dataset9"};

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
  Handler() = default;

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
      size_t,
      const unsigned short number_of_robots = SimulationDefaults::kRobots,
      const unsigned short number_of_landmarks = SimulationDefaults::kLandmarks,
      const double sample_period = SimulationDefaults::kSamplePeriod,
      const std::string &output_directory = SimulationDefaults::kOutputDir,
      const unsigned long seed = SimulationDefaults::kSeed);

  [[nodiscard]] std::string getDatasetName();
  [[nodiscard]] std::string getDataExtractionDirectory();
  [[nodiscard]] std::string getDataInferenceDirectory();

  [[nodiscard]] Robot::List &getRobots();
  [[nodiscard]] Robot &getRobot(Agent::ID id);

  [[nodiscard]] const Landmark::List &getLandmarks() const;
  [[nodiscard]] Landmark &getLandmark(Agent::ID id);

  [[nodiscard]] const double getSamplePeriod() const;

  [[nodiscard]] const unsigned short getNumberOfRobots() const;
  [[nodiscard]] const unsigned short getNumberOfLandmarks() const;
  [[nodiscard]] const unsigned short getNumberOfBarcodes() const;
  [[nodiscard]] const size_t getNumberOfSyncedDatapoints() const;
  [[nodiscard]] const std::map<Agent::ID, size_t> &
  getNumberOfSyncedMeasurements() const;
  [[nodiscard]] const size_t getNumberOfSyncedMeasurements(Agent::ID id) const;

  [[nodiscard]] static const size_t
  getNumberOfMeasurements(const std::vector<Robot::Measurement> &);

  [[nodiscard]] const Agent *getAgent(const Agent::Barcode &) const;

  void saveExtractedData();
  void saveInferenceData();

  void calculateStateError();
  Robot::State getAverageRMSE();

  static const std::string getProjectDirectory();

  /** Folder in which the output data is saved. */
  static constexpr const char *output_folder{"/output/"};

private:
  /**
   * @brief Folder location for the dataset.
   */
  std::string dataset_{};

  /**
   * @brief Folder location of the output directory inside the "./output/"
   * folder.
   */
  std::string output_directory_{};

  /**
   * @brief Folder location for the output data correspoding the data
   * extraction process.
   */
  std::string data_extraction_directory_{};

  /**
   * @brief Folder location for the output data corresponding to the infernce
   * peformance.
   */
  std::string data_inference_directory_{};

  /**
   * @brief the desired sample period for resampling the data to sync the
   * timesteps between the vehicles.
   */
  double sampling_period_{0.2};

  /**
   * @brief The total number of landmarks in the dataset.
   */
  unsigned short total_landmarks_{};

  /**
   * @brief The total number of robots in the dataset.
   */
  unsigned short total_robots_{};

  /**
   * @brief the total number of barcodes in the dataset.
   * @note the value of this variable is the summation of the
   * Data::Handler::TOTAL_LANDMARKS and Data::Handler::TOTAL_ROBOTS.
   */
  unsigned short total_barcodes_{};

  /**
   * @brief The number of datapoints after calling Robot::sync_data.
   */
  unsigned long total_synced_datapoints_{};

  /**
   * @brief the total number of synced measurements for each robot.
   */
  std::map<Agent::ID, size_t> total_synced_measurements_;

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
   * @brief Simulator class responsible for creating odometry, and measurement
   * data for the robots, and assigning positions to the landmarks.
   */
  Simulator simulator_;

  /**
   * Flag indicating whether the data handler is currently using simulated data.
   */
  bool simulation_{};

  void setOutputDirectory(const std::string &, const std::string &);

  void readBarcodes(const std::string &);
  void readLandmarks(const std::string &);
  void readGroundTruth(const std::string &, const Agent::ID &);
  void readOdometry(const std::string &, const Agent::ID &);
  void readMeasurements(const std::string &, const Agent::ID &);

  void syncData(const double &);

  void calculateGroundtruthOdometry();
  void calculateGroundtruthMeasurement();

  void setNumberOfSyncedMeasurements();

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
} // namespace utias::mrclam
#endif // INCLUDE_INCLUDE_DATA_EXTRACTOR_H_
