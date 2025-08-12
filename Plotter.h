/**
 * @file Plotter.h
 * @brief Contains gnuplot functionality.
 * @date 2025-08-06
 */
#pragma once

#include "DataHandler.h"
#include "Landmark.h"

#include <cmath>
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

  void demo_animation();
  void plotGroundruthStates(unsigned short robot_id = 0);
  void plotGroundruthTrajectory(unsigned short robot_id = 0);
  void plotOdometry(unsigned short robot_id = 0);
  void plotMeasurements(unsigned short robot_id = 0);

private:
  class Plot;

  /** Reference to a data handler instance */
  Handler &data_;

  std::string dataset_name_;

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
    ORIENTATION = 4

  };

  /**
   * @struct RobotData
   * Houses the serialised data fields for all information pertaining to the
   * robots.
   */
  struct RobotData {

    struct Types {
      std::string raw;
      std::string synced;
      std::string groundtruth;
      std::string error;
      const std::string binary_format;
    };

    Types odometry{.binary_format = " binary format='%double%double%double' "};

    Types measurement{.binary_format =
                          " binary format='%double%ushort%double%double' "};

    Types pose{.binary_format =
                   " binary format='%double%double%double%double' "};
  };

  /** Filename of the binary landmark data. */
  struct {
    std::string filename;
    const std::string binary_format = " binary format='%double%double' ";
  } binary_landmark_data_;

  /** Vector containing the serialised data extracted into the RobotData struct.
   */
  std::vector<RobotData> binary_robot_data_;

  struct Plot {
    std::string binary_name;
    std::string binary_format;

    gnuplot::PlotSettings settings;

    Plot(const std::string binary_name, std::string binary_format)
        : binary_name(binary_name), binary_format(binary_format) {}
  };

  void plot(const PlotList &, const gnuplot::AxisSettings &);

  void binariseRobotPoseData(unsigned short);
  void binariseRobotInferenceData(unsigned short);
  void binariseLandmarkData();
  void binariseOdometryData(unsigned short);
  void binariseMeasurementData(unsigned short);

  void write_binary(std::string &, const std::vector<Robot::Odometry> &);
  void write_binary(std::string &, const std::vector<Robot::State> &);
  void write_binary(std::string &, const std::vector<Robot::Measurement> &);
  void write_binary(std::string &, const std::vector<Landmark> &);
};

} // namespace Data
