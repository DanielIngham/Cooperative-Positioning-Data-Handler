#include "Plotter.h"
#include "Landmark.h"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace Data {

Plotter::Plotter(Handler &data)
    : data_(data), dataset_name_(data_.getDatasetName()),
      data_points_(data_.getNumberOfSyncedDatapoints()),
      total_measurements_(data_.getNumberOfSyncedMeasurements()),
      total_robots_(data_.getNumberOfRobots()),
      total_landmarks_(data_.getNumberOfLandmarks()) {

  std::vector<Robot> &input_robot_data = data_.getRobots();

  binary_robot_data_.resize(total_robots_);
}

/**
 * Default destructor.
 */
Plotter::~Plotter() {}

/**
 * Converts the Robot data structure into serial tuples for plotting
 * using gnuplot.
 */
void Plotter::binariseRobotPoseData(unsigned short robot_id) {
  std::vector<Robot> &input_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {

    /* Serialise state data. */
    const std::vector<Robot::State> &groundtruth_states =
        input_robot_data[id].groundtruth.states;

    std::string &filename = binary_robot_data_[id].pose.groundtruth;

    filename = dataset_name_ + "_Robot_" + std::to_string(id + 1) +
               "_groundtruth" + "_pose";

    write_binary(filename, groundtruth_states);

    /* TODO: Add Raw . */
  }
}

/**
 * Covert robot odometry sensor values to temporary binary files for gnuplot.
 */
void Plotter::binariseOdometryData(unsigned short robot_id) {
  std::vector<Robot> &input_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {

    /* Binarise synced odometry. */
    const std::vector<Robot::Odometry> &synced_odometry =
        input_robot_data[id].synced.odometry;

    binary_robot_data_[id].odometry.synced = dataset_name_ + "_Robot_" +
                                             std::to_string(id + 1) +
                                             "_synced" + "_odometry";

    write_binary(binary_robot_data_[id].odometry.synced, synced_odometry);

    /* Binarise groundtruth odometry. */
    const std::vector<Robot::Odometry> &groundtruth_odometry =
        input_robot_data[id].groundtruth.odometry;

    binary_robot_data_[id].odometry.groundtruth = dataset_name_ + "_Robot_" +
                                                  std::to_string(id + 1) +
                                                  "_groundtruth" + "_odometry";

    write_binary(binary_robot_data_[id].odometry.groundtruth,
                 groundtruth_odometry);

    /* TODO: binarise raw odometry */
  }
}

void Plotter::binariseMeasurementData(unsigned short robot_id) {
  std::vector<Robot> &output_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
  }
}

void Plotter::binariseRobotInferenceData(unsigned short robot_id) {

  std::vector<Robot> &output_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {

    const std::vector<Robot::State> &estimated_states =
        output_robot_data[id].synced.states;

    binary_robot_data_[id].pose.synced = dataset_name_ + "_Robot_" +
                                         std::to_string(id + 1) + "_estimated" +
                                         "_odometry";

    write_binary(binary_robot_data_[id].pose.synced, estimated_states);
  }
}

/**
 * Converts the Landmark data structure into serial tuples for plotting
 * using gnuplot.
 */
void Plotter::binariseLandmarkData() {

  const std::vector<Landmark> &input_landmark_data = data_.getLandmarks();

  binary_landmark_data_.filename = dataset_name_ + "_Landmarks";

  write_binary(binary_landmark_data_.filename, input_landmark_data);
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
  if (robot_id > total_robots_ || robot_id <= 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseRobotPoseData(robot_id);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Groundtruth";
    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::grid();
    gnuplot_ << gnuplot::setMultiplot(3, 1);

    /* Plot X Position */
    PlotList x_plots;
    x_plots.emplace_back(binary_robot_data_[id].pose.groundtruth,
                         binary_robot_data_[id].pose.binary_format);

    x_plots.back().settings.title = "Groundtruth";

    gnuplot::AxisSettings x_position_axis;
    x_position_axis.x_label = "Time [s]";
    x_position_axis.y_label = "X Position [m]";

    plot(x_plots, x_position_axis);

    /* Plot Y Position */
    PlotList y_plots;
    y_plots.emplace_back(binary_robot_data_[id].pose.groundtruth,
                         binary_robot_data_[id].pose.binary_format);
    y_plots.back().settings.title = "Groundtruth";
    y_plots.back().settings.y = Y_POSITION;

    gnuplot::AxisSettings y_position_axis;
    y_position_axis.x_label = "Time [s]";
    y_position_axis.y_label = "X Position [m]";

    plot(y_plots, y_position_axis);

    /* Plot Orientation Position */
    PlotList orientation_plots;
    orientation_plots.emplace_back(binary_robot_data_[id].pose.groundtruth,
                                   binary_robot_data_[id].pose.binary_format);

    orientation_plots.back().settings.title = "Groundtruth";
    orientation_plots.back().settings.y = ORIENTATION;

    gnuplot::AxisSettings orientation_axis;
    orientation_axis.x_label = "Time [s]";
    orientation_axis.y_label = "X Position [m]";

    plot(orientation_plots, orientation_axis);

    gnuplot_ << gnuplot::unsetMultiplot();

    gnuplot_.flush();
  }
}

/**
 * Plots the xy trajectory of the robots alongside the positions of the
 * landmarks.
 * @param robot_id Identifier of the robot whose data you want to plot. If the
 * value is 0, then all robots data is plotted.
 */
void Plotter::plotGroundruthTrajectory(unsigned short robot_id) {
  if (robot_id > total_robots_ || robot_id <= 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseLandmarkData();
  binariseRobotPoseData(robot_id);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Trajectory";
    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::grid();

    PlotList plots;
    plots.emplace_back(binary_landmark_data_.filename,
                       binary_landmark_data_.binary_format);

    plots.back().settings.style = gnuplot::PlotStyle::POINTS;
    plots.back().settings.title = "Landmarks";

    /* Create the plot for the trajectory of the robots. */
    plots.emplace_back(binary_robot_data_[id].pose.groundtruth,
                       binary_robot_data_[id].pose.binary_format);

    plots.back().settings.x = X_POSITION;
    plots.back().settings.y = Y_POSITION;
    plots.back().settings.style = gnuplot::PlotStyle::LINES;
    plots.back().settings.title = "Robot Trajectory";

    gnuplot::AxisSettings axis;

    axis.x_label = "x position [m]";
    axis.y_label = "y position [m]";

    plot(plots, axis);

    gnuplot_.flush();
  }
}

/**
 *
 */
void Plotter::plotOdometry(unsigned short robot_id) {
  if (robot_id > total_robots_ || robot_id <= 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseOdometryData(robot_id);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Odometry";
    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::setMultiplot(2, 1);
    gnuplot_ << gnuplot::grid();

    /* Set forward velocity axis labels */
    gnuplot::AxisSettings forward_velocity_axis;

    forward_velocity_axis.y_label = "Forward Velocity [m/s]";
    forward_velocity_axis.x_label = "Time [s]";

    /* Forward Velcoty Plot */
    PlotList forward_velocity_plots;

    forward_velocity_plots.emplace_back(
        binary_robot_data_[id].odometry.groundtruth,
        binary_robot_data_[id].odometry.binary_format);

    forward_velocity_plots.back().settings.title = "Groundtruth";

    forward_velocity_plots.emplace_back(
        binary_robot_data_[id].odometry.synced,
        binary_robot_data_[id].odometry.binary_format);

    forward_velocity_plots.back().settings.title = "synced";
    forward_velocity_plots.back().settings.linecolor = gnuplot::Colour::RED;

    plot(forward_velocity_plots, forward_velocity_axis);

    /* Angular Velcoty Plot */
    PlotList angular_velocity_plots;

    angular_velocity_plots.emplace_back(
        binary_robot_data_[id].odometry.groundtruth,
        binary_robot_data_[id].odometry.binary_format);

    angular_velocity_plots.back().settings.title = "Groundtruth";
    angular_velocity_plots.back().settings.y = ANGULAR_VELOCITY;

    angular_velocity_plots.emplace_back(
        binary_robot_data_[id].odometry.synced,
        binary_robot_data_[id].odometry.binary_format);

    angular_velocity_plots.back().settings.title = "synced";
    angular_velocity_plots.back().settings.y = ANGULAR_VELOCITY;
    angular_velocity_plots.back().settings.linecolor = gnuplot::Colour::RED;

    gnuplot::AxisSettings angular_velocity_axis;

    angular_velocity_axis.y_label = "Angular Velocity [rad/s]";
    angular_velocity_axis.x_label = "Time [s]";

    plot(angular_velocity_plots, angular_velocity_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

void Plotter::write_binary(std::string &filename,
                           const std::vector<Robot::Odometry> &odometry_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

  if (std::filesystem::exists(filename)) {
    return;
  }

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  for (auto const &row : odometry_data) {
    fout.write(reinterpret_cast<const char *>(&row.time), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&row.forward_velocity),
               sizeof(double));
    fout.write(reinterpret_cast<const char *>(&row.angular_velocity),
               sizeof(double));
  }

  fout.close();
}

void Plotter::write_binary(std::string &filename,
                           const std::vector<Robot::State> &state_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

  if (std::filesystem::exists(filename)) {
    return;
  }

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  for (auto const &row : state_data) {
    fout.write(reinterpret_cast<const char *>(&row.time), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&row.x), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&row.y), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&row.orientation),
               sizeof(double));
  }

  fout.close();
}

void Plotter::write_binary(
    std::string &filename,
    const std::vector<Robot::Measurement> &measurement_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

  if (std::filesystem::exists(filename)) {
    return;
  }

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  for (auto const &row : measurement_data) {
    for (unsigned short i = 0; i < row.subjects.size(); ++i) {
      fout.write(reinterpret_cast<const char *>(&row.time), sizeof(double));
      fout.write(reinterpret_cast<const char *>(&row.subjects),
                 sizeof(unsigned short));
      fout.write(reinterpret_cast<const char *>(&row.ranges[i]),
                 sizeof(double));
      fout.write(reinterpret_cast<const char *>(&row.bearings[i]),
                 sizeof(double));
    }
  }

  fout.close();
}

void Plotter::write_binary(std::string &filename,
                           const std::vector<Landmark> &landmark_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

  if (std::filesystem::exists(filename)) {
    return;
  }

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  for (auto const &row : landmark_data) {
    fout.write(reinterpret_cast<const char *>(&row.x), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&row.y), sizeof(double));
  }

  fout.close();
}

/**
 *
 */
void Plotter::plot(const PlotList &plots,
                   const gnuplot::AxisSettings &axis_settings) {

  size_t total_plots = plots.size();

  /* Set the axis settings. */
  gnuplot_ << gnuplot::setAxisSettings(axis_settings);

  gnuplot_ << "plot";

  for (size_t i = 0; i < total_plots; ++i) {
    assert(plots[i].binary_name != "" && "Dataset name not set.");

    gnuplot_ << " '" << plots[i].binary_name << "' " << plots[i].binary_format
             << gnuplot::setPlotSettings(plots[i].settings);

    if (i < total_plots - 1) {
      gnuplot_ << ",";
    }
  }

  gnuplot_ << "\n";
}

} // namespace Data
