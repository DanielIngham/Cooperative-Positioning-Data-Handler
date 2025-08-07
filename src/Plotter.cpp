#include "Plotter.h"
#include "Landmark.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace Data {

Plotter::Plotter(Handler &data)
    : data_(data), data_points_(data_.getNumberOfSyncedDatapoints()),
      total_measurements_(data_.getNumberOfSyncedMeasurements()),
      total_robots_(data_.getNumberOfRobots()),
      total_landmarks_(data_.getNumberOfLandmarks()) {

  /* Assign memory to each of the tuples. */
  serial_robot_data_.resize(total_robots_);
  serial_landmark_data_.resize(total_landmarks_);

  for (unsigned short id = 0; id < total_robots_; ++id) {

    serial_robot_data_[id].odometry.resize(data_points_);

    serial_robot_data_[id].measurement.resize(data_points_);

    serial_robot_data_[id].pose.estimate.resize(data_points_);
    serial_robot_data_[id].pose.groundtruth.resize(data_points_);
  }

  serialiseRobotInputData();
  serialiseLandmarkData();
}

/**
 * Default destructor.
 */
Plotter::~Plotter() {}

/**
 * Converts the Robot data structure into serial tuples for plotting
 * using gnuplot.
 */
void Plotter::serialiseRobotInputData() {
  std::vector<Robot> &input_robot_data = data_.getRobots();

  for (unsigned short id = 0; id < total_robots_; ++id) {

    const std::vector<Robot::State> &groundtruth_states =
        input_robot_data[id].groundtruth.states;

    /* Serialise the robot data structure. */
    std::transform(groundtruth_states.begin(), groundtruth_states.end(),
                   serial_robot_data_[id].pose.groundtruth.begin(),
                   [](const Robot::State &state) {
                     return std::make_tuple(state.time, state.x, state.y,
                                            state.orientation);
                   });
  }
}

void Plotter::serialiseRobotOutputData() {

  std::vector<Robot> &output_robot_data = data_.getRobots();

  for (unsigned short id = 0; id < total_robots_; ++id) {

    const std::vector<Robot::State> &estimated_states =
        output_robot_data[id].synced.states;

    /* Serialise the robot data structure. */
    std::transform(estimated_states.begin(), estimated_states.end(),
                   serial_robot_data_[id].pose.estimate.begin(),
                   [](const Robot::State &state) {
                     return std::make_tuple(state.time, state.x, state.y,
                                            state.orientation);
                   });
  }
}

/**
 * Converts the Landmark data structure into serial tuples for plotting
 * using gnuplot.
 */
void Plotter::serialiseLandmarkData() {
  const std::vector<Landmark> &input_landmark_data = data_.getLandmarks();

  std::transform(input_landmark_data.begin(), input_landmark_data.end(),
                 serial_landmark_data_.begin(), [](const Landmark &landmark) {
                   return std::make_tuple(landmark.x, landmark.y);
                 });
}

void Plotter::demo_animation() {

  std::cout << "Press Ctrl-C to quit (closing gnuplot window doesn't quit)."
            << std::endl;

  gnuplot_ << "set yrange [-1:1]\n";

  const int N = 1000;
  std::vector<double> pts(N);

  double theta = 0;
  while (1) {
    for (int i = 0; i < N; i++) {
      double alpha = (static_cast<double>(i) / N - 0.5) * 10;
      pts[i] = sin(alpha * 8.0 + theta) * exp(-alpha * alpha / 2.0);
    }

    gnuplot_ << "plot '-' binary" << gnuplot_.binFmt1d(pts, "array")
             << "with lines notitle\n";
    gnuplot_.sendBinary1d(pts);
    gnuplot_.flush();

    theta += 0.2;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

/**
 * Plots the groundtruth states of the robots.
 * @param robot_id The id of the robot whose states will be plotted.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotGroundruthStates(unsigned short robot_id) {

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Groundtruth";

    gnuplot_ << setTerminal(id);
    gnuplot_ << "set grid\n";
    gnuplot_ << setMultiplot(3, 1, title);

    gnuplot_ << "$groundtruth_pose << EOD\n";
    gnuplot_.send1d(serial_robot_data_[id].pose.groundtruth);
    gnuplot_ << "EOD\n";

    gnuplot_ << "$raw_pose << EOD\n";
    gnuplot_.send1d(serial_robot_data_[id].pose.groundtruth);
    gnuplot_ << "EOD\n";

    /* Let style */
    gnuplot_ << "set style data points\n";
    gnuplot_ << "set pointsize 0.1\n";

    gnuplot_ << "plot $raw_pose using 1:2 title 'Raw' lc rgb 'red' pt 7,"
             << "$groundtruth_pose using 1:2 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_ << "plot $raw_pose using 1:3 title 'Raw' lc rgb 'red' pt 7,"
             << "$groundtruth_pose using 1:3 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_ << "plot $raw_pose using 1:4 title 'Raw' lc rgb 'red' pt 7,"
             << "$groundtruth_pose using 1:4 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_ << unsetMultiplot();

    gnuplot_.flush();
  }
}

void Plotter::plotGroundruthTrajectory(unsigned short robot_id) {

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Groundtruth";

    gnuplot_ << setTerminal(id);
    gnuplot_ << "set grid\n";

    gnuplot_ << "$groundtruth_pose << EOD\n";
    gnuplot_.send1d(serial_robot_data_[id].pose.groundtruth);
    gnuplot_ << "EOD\n";

    gnuplot_ << "$raw_pose << EOD\n";
    gnuplot_.send1d(serial_robot_data_[id].pose.groundtruth);
    gnuplot_ << "EOD\n";

    gnuplot_ << "$landmarks << EOD\n";
    gnuplot_.send1d(serial_landmark_data_);
    gnuplot_ << "EOD\n";

    gnuplot_ << "set style data line\n";
    gnuplot_ << "set pointsize 0.1\n";

    gnuplot_ << "plot $landmarks using 1:2 with points pointsize 1.0 pointtype "
                "6 title 'Landmarks',"
             << "$groundtruth_pose using 2:3 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_.flush();
  }
}

std::string Plotter::setTerminal(unsigned short terminal_number) {
  std::string terminal_type = " qt ";
  std::string terminal_size = " size 1336,768 ";

  std::string terminal_settings = "";
  terminal_settings += " set mouse\n";
  terminal_settings += " set term " + terminal_type +
                       std::to_string(terminal_number) + terminal_size +
                       " noraise\n";
  terminal_settings += "set samples 1000\n";

  return terminal_settings;
}

std::string Plotter::setMultiplot(unsigned short rows, unsigned short columns,
                                  const std::string title) {
  //      set multiplot layout 3,1 title sprintf("Robot %d Groundtruth
  // Trajectory", i)
  std::string multi_plot_settings = "set multiplot ";

  /* Adding layout constraints */
  multi_plot_settings +=
      "layout " + std::to_string(rows) + "," + std::to_string(columns) + " ";

  if (title != "") {
    multi_plot_settings += "title \"" + title + '"';
  }

  multi_plot_settings += '\n';

  return multi_plot_settings;
}

std::string Plotter::unsetMultiplot() { return "unset multiplot\n"; }

} // namespace Data
