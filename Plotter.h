/**
 * @file Plotter.h
 * @brief Contains gnuplot functionality.
 * @date 2025-08-06
 */
#pragma once

#include "DataHandler.h"

#include <cmath>
#include <tuple>
#include <unistd.h>
#include <vector>

/* Warn about depreciated functions. */
#define GNUPLOT_DEPRECATE_WARN
#include "./external/gnuplot/gnuplot-iostream.h"

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

private:
  /** Reference to a data handler instance */
  Handler &data_;

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
  typedef std::vector<
      std::tuple<double /* time [s] */, double /* forward velocity [m/s]*/,
                 double /* Angular velocity [rad/s]*/>>
      odometry_tuple;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the robots poses. The vector will be of size total_robots.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - double : global x position [m]
   *   - double : global y position [m]
   *   - double : global orientation [rad]
   */
  typedef std::vector<
      std::tuple<double /* time [s] */, double /* x position [m] */,
                 double /* y position [m] */, double /* orientation [rad] */>>
      pose_tuple;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the landmarks points. The vector will be of size total_landmarks.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - double : global x position [m]
   *   - double : global y position [m]
   */
  typedef std::vector<
      std::tuple<double /* x position [m] */, double /* y position [m] */>>
      point_tuple;

  /**
   * Type defintion for the vector of tuples containing the serialised data of
   * the landmarks points. The vector will be of size total_landmarks.
   * The elements in the vector correspond to:
   *   - double : time [s]
   *   - unsigned short: Subject ID
   *   - double : relative range to agent measured [m]
   *   - double : relative bearing to agent measured [rad]
   */
  typedef std::vector<
      std::tuple<double /* time [s] */, unsigned short /* Subject */,
                 double /* range [m] */, double /* bearing [rad] */>>
      measurement_tuple;

  /**
   * @struct RobotData
   * Houses the serialised data fields for all information pertaining to the
   * robots.
   */
  struct RobotData {

    struct {
      odometry_tuple interpolated;
      odometry_tuple groundtruth;
      odometry_tuple raw;
    } odometry;

    struct {
      measurement_tuple interpolated;
      measurement_tuple raw;

    } measurement;

    struct {
      pose_tuple estimate;
      pose_tuple groundtruth;
      pose_tuple raw;
    } pose;
  };

  /** Vector containing the serialised data extracted into the RobotData struct.
   */
  std::vector<RobotData> serial_robot_data_;

  /** Vector containing the global points of each landmark. */
  point_tuple serial_landmark_data_;

  void serialiseRobotInputData();
  void serialiseRobotOutputData();

  void serialiseLandmarkData();

  std::string setTerminal(unsigned short terminal_number = 0);

  std::string setMultiplot(unsigned short, unsigned short,
                           const std::string title = "");
  std::string unsetMultiplot();
};

} // namespace Data
