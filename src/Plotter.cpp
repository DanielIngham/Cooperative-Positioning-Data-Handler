#include "Plotter.h"
#include "Landmark.h"

#include <algorithm>
#include <chrono>
#include <iostream>
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
  /* The landmark data should be constant for a data set, therefore it only
   * needs to be serialised in once. */
  if (!serial_landmark_data_.empty())
    return;

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

void Plotter::plotGroundruth(unsigned short robot_id) {

  gnuplot_ << "set mouse\n"
           << "set term wxt noraise\n"
           << "set samples 1000\n";

  gnuplot_ << "plot '-' using 1:1 with lines title 'xy plot'\n";
  gnuplot_.send1d(serial_robot_data_[robot_id].pose.groundtruth);

  gnuplot_.flush();
}

} // namespace Data
