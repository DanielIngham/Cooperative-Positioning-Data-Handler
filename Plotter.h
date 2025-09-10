/**
 * @file Plotter.h
 * @brief Contains gnuplot functionality.
 * @date 2025-08-06
 */
#pragma once

#include "DataHandler.h"
#include "Landmark.h"
#include "Robot.h"

#include <cmath>
#include <initializer_list>
#include <tuple>
#include <unistd.h>
#include <variant>
#include <vector>

/* Warn about depreciated functions. */
#define GNUPLOT_DEPRECATE_WARN
/* TODO: Fix the include path in CMake. */
#include "./external/gnuplot/gnuplot-iostream.h"
#include "./external/gnuplot/gnuplot_helper.h"

namespace Data {

class Plotter {
public:
  explicit Plotter(Handler &);
  Plotter(Plotter &&) = delete;
  Plotter(const Plotter &) = delete;
  Plotter &operator=(Plotter &&) = delete;
  Plotter &operator=(const Plotter &) = delete;
  ~Plotter();

  /** Type of data extracted/calculated from the dataset. */
  enum PlotType {
    SYNCED,      ///< Synced data after linear interpolation.
    RAW,         ///< Raw data extracted from the dataset.
    GROUNDTRUTH, ///< Groundtruth data extracted or calculated from the dataset.
    ERROR,       ///< Difference between the groundtruth and the synced data.
    ABSOLUTE_ERROR ///< Absolute value of the error.
  };

  void setTerminal(gnuplot::TerminalSettings);

  void plotPoses(std::initializer_list<PlotType>, unsigned short robot_id = 0);

  void plotTrajectory(std::initializer_list<PlotType>,
                      unsigned short robot_id = 0);

  void plotOdometry(std::initializer_list<PlotType>,
                    unsigned short robot_id = 0);

  void plotMeasurements(std::initializer_list<PlotType>,
                        unsigned short robot_id = 0);

  void plotMeasurementsVector(std::initializer_list<PlotType>,
                              unsigned short robot_id = 0);

  void plotOdometryPDFs(unsigned short robot_id = 0,
                        const double bin_size = 0.001);

  void plotMeasurementPDFs(unsigned short robot_id = 0,
                           const double bin_size = 0.001);

  void addInferenceIteration();

  void inference_error_animation(std::initializer_list<PlotType>);

  void plotInferenceIterations(std::initializer_list<PlotType>,
                               std::vector<unsigned int> iterations = {});

private:
  struct Plot {
    bool using_binary_file{};
    std::string binary_name;
    std::string binary_format;

    gnuplot::PlotSettings settings;

    std::string plot_string;

    Plot(const std::string binary_name, std::string binary_format)
        : binary_name(binary_name), binary_format(binary_format) {
      using_binary_file = true;
    }

    Plot(const std::string plot_string) : plot_string(plot_string) {}
  };

  /** Data structure containing the settings of a gnuplot terminal. */
  gnuplot::TerminalSettings terminal_;

  /** The number of terminals that have been spawned by the
   * plotter. */
  static unsigned short terminal_number_;

  unsigned int total_inference_iterations_{};

  /** Reference to a data handler instance */
  Handler &data_;

  /** Name of the dataset that the datahandler is using. */
  std::string dataset_name_;

  /** Directory where the data extraction plots are saved. */
  std::string data_extraction_directory_;

  /** Directory where the data inference plots are saved. */
  std::string data_infernce_directory_;

  /** Total number of synced data points in the dataset. */
  size_t data_points_;

  /** Total number of synced measurements present for each robot in the dataset.
   */
  std::vector<size_t> total_measurements_;

  /** Total number of robots present in the dataset. */
  unsigned short total_robots_;

  /** Total number of landmarks present in the dataset. */
  unsigned short total_landmarks_;

  /** Instance of the gnuplot iostream class that allows for plotting */
  Gnuplot gnuplot_;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the robots odometry inputs. The vector will be of size total_robots.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - double : forward velocity [m/s]
   *   - double : angular velocity [rad/s]
   */
  using odometry_tuple = std::vector<
      std::tuple<double /* time [s] */, double /* forward velocity [m/s]*/,
                 double /* Angular velocity [rad/s]*/>>;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the robots poses. The vector will be of size total_robots.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - double : global x position [m]
   *   - double : global y position [m]
   *   - double : global orientation [rad]
   */
  using pose_tuple = std::vector<
      std::tuple<double /* time [s] */, double /* x position [m] */,
                 double /* y position [m] */, double /* orientation [rad] */>>;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the landmarks points. The vector will be of size total_landmarks.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - double : global x position [m]
   *   - double : global y position [m]
   */
  using point_tuple = std::vector<
      std::tuple<double /* x position [m] */, double /* y position [m] */>>;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the landmarks points. The vector will be of size total_landmarks.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - unsigned short: Subject ID
   *   - double : relative range to agent measured [m]
   *   - double : relative bearing to agent measured [rad]
   */
  using measurement_tuple = std::vector<
      std::tuple<double /* time [s] */, unsigned short /* Subject */,
                 double /* range [m] */, double /* bearing [rad] */>>;

  using PlotData =
      std::variant<pose_tuple, point_tuple, measurement_tuple, odometry_tuple>;

  using PlotList = std::vector<Plot>;

  /** Indices of the items in the tuples used for gnuplotting.
   * @note gnuplot starts its indexing at 1. */
  enum tuples_index_plot {
    TIME = 1,
    /* Odometry */
    FORWARD_VELOCITY = 2,
    ANGULAR_VELOCITY = 3,
    /* Measurements */
    SUBJECT = 2,
    RANGE = 3,
    BEARING = 4,
    /* Pose/ Point */
    X_POSITION = 2,
    Y_POSITION = 3,
    ORIENTATION = 4,
    /* PDF */
    BIN_INDEX = 1,
    BIN_WIDTH = 2,
    BIN_COUNT = 3,
  };

  /**
   * @struct RobotData
   * Houses the serialised data fields for all information pertaining to the
   * robots.
   */
  struct RobotData {

    /**
     * @struct PDF
     * Contains the name and binary format of data file containing the bins of
     * the Probability Density Function .
     */
    struct PDF {
      std::string filename;
      static constexpr const char *binary_format = "%double%double%double";
    } forward_velocity_pdf, angular_velocity_pdf, range_pdf, bearing_pdf;

    /**
     * @struct MeasurementVector
     * Contains the vectors of measurements taken by the robot. For example,
     * given the range and bearing, a vector can be drawn from the robots
     * current position to a given item measured.
     */
    struct MeasurementVector {
      std::string filename;
      static constexpr const char *binary_format =
          "%double%double%double%double";
    } measurement_vector;

    /**
     * @struct Types
     * Contains the names and binary format of data files associated with the
     * different types of data structures.
     */
    struct DataVariants {
      std::string raw;         ///< Raw data.
      std::string synced;      ///< Data synced by linear interpolation.
      std::string groundtruth; ///< Groundtruth data.
      std::string error;       ///< Error data (synced - groundtruth)

      virtual ~DataVariants() = default;
    };

    /** Contains the filenames of the variants of extracted odometry data. */
    struct Odometry : DataVariants {
      static std::string binary_format() { return "%double%double%double"; };
    } odometry;

    /** Contains the filenames of the variants of extracted measurement data. */
    struct Measurement : DataVariants {
      static std::string binary_format() {
        return "%double%ushort%double%double";
      }
    } measurement;

    /** Contains the filenames of the variants of extracted Pose data. */
    struct Pose : DataVariants {
      static std::string binary_format() {
        return "%double%double%double%double";
      }
    } pose;

    std::string absolute_pose_error; ///< Absolute value of error data.

    struct Iterations {
      std::string pose;
      std::string error;
      std::string absolute_error;
    } iterations;
  };

  /** Filename of the binary landmark data. */
  struct {
    std::string filename;
    const std::string binary_format = "%double%double";
  } binary_landmark_data_;

  void plot(const PlotList &, const gnuplot::AxisSettings &);

  /** Vector containing the serialised data extracted into the RobotData struct.
   */
  std::vector<RobotData> binary_robot_data_;

  void inferenceIterationsPlotter(PlotType,
                                  std::vector<unsigned int> iteration_numbers);

  void binariseRobotPoseData(std::initializer_list<PlotType>, unsigned short);
  void binariseLandmarkData();
  void binariseOdometryData(std::initializer_list<PlotType>, unsigned short);
  void binariseMeasurementData(std::initializer_list<PlotType>, unsigned short);
  void binariseMeasurementVectors(std::initializer_list<PlotType>,
                                  unsigned short);

  void binariseOdometryPDF(unsigned short, const double);
  void binariseMeasurementPDF(unsigned short, const double);

  void write_binary(std::string &, const std::vector<Robot::Odometry> &);
  void write_binary(std::string &, const std::vector<Robot::State> &,
                    bool append = false);
  void write_binary(std::string &, const std::vector<Robot::Measurement> &);
  void write_binary(std::string &, const std::vector<Landmark> &);
  void write_binary(std::string &, const std::unordered_map<int, double> &,
                    const double &);
};

} // namespace Data
