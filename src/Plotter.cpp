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

  std::vector<Robot> &input_robot_data = data_.getRobots();

  /* Assign memory to each of the tuples. */
  serial_robot_data_.resize(total_robots_);
  serial_landmark_data_.resize(total_landmarks_);

  for (unsigned short id = 0; id < total_robots_; ++id) {
    serial_robot_data_[id].odometry.interpolated.resize(data_points_);
    serial_robot_data_[id].odometry.groundtruth.resize(data_points_);

    serial_robot_data_[id].measurement.interpolated.resize(
        total_measurements_[id]);

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

    /* Serialise state data. */
    const std::vector<Robot::State> &groundtruth_states =
        input_robot_data[id].groundtruth.states;

    std::transform(groundtruth_states.begin(), groundtruth_states.end(),
                   serial_robot_data_[id].pose.groundtruth.begin(),
                   [](const Robot::State &state) {
                     return std::make_tuple(state.time, state.x, state.y,
                                            state.orientation);
                   });

    /* Serialise odometry */
    const std::vector<Robot::Odometry> &interpolated_odometry =
        input_robot_data[id].synced.odometry;

    std::transform(interpolated_odometry.begin(), interpolated_odometry.end(),
                   serial_robot_data_[id].odometry.interpolated.begin(),
                   [](const Robot::Odometry &odometry) {
                     return std::make_tuple(odometry.time,
                                            odometry.forward_velocity,
                                            odometry.angular_velocity);
                   });

    const std::vector<Robot::Odometry> &groundtruth_odometry =
        input_robot_data[id].groundtruth.odometry;

    std::transform(groundtruth_odometry.begin(), groundtruth_odometry.end(),
                   serial_robot_data_[id].odometry.groundtruth.begin(),
                   [](const Robot::Odometry &odometry) {
                     return std::make_tuple(odometry.time,
                                            odometry.forward_velocity,
                                            odometry.angular_velocity);
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
    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::grid();
    gnuplot_ << gnuplot::setMultiplot(3, 1);

    /* Plot X Position */
    PlotList x_plots;
    x_plots.emplace_back(serial_robot_data_[id].pose.groundtruth);
    x_plots[0].settings.title = "Groundtruth";

    gnuplot::AxisSettings x_position_axis;
    x_position_axis.x_label = "Time [s]";
    x_position_axis.y_label = "X Position [m]";

    plot(x_plots, x_position_axis);

    /* Plot Y Position */
    PlotList y_plots;
    y_plots.emplace_back(serial_robot_data_[id].pose.groundtruth);
    y_plots.back().settings.title = "Groundtruth";
    y_plots.back().settings.y = Y_POSITION;

    gnuplot::AxisSettings y_position_axis;
    y_position_axis.x_label = "Time [s]";
    y_position_axis.y_label = "X Position [m]";

    plot(y_plots, y_position_axis);

    /* Plot Orientation Position */
    PlotList orientation_plots;
    orientation_plots.emplace_back(serial_robot_data_[id].pose.groundtruth);
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

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Trajectory";
    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::grid();

    PlotList plots;
    plots.emplace_back(serial_landmark_data_);
    plots.back().settings.style = gnuplot::PlotStyle::POINTS;
    plots.back().settings.title = "Landmarks";

    /* Create the plot for the trajectory of the robots. */
    plots.emplace_back(serial_robot_data_[id].pose.groundtruth);
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
        serial_robot_data_[id].odometry.groundtruth);
    forward_velocity_plots.back().settings.title = "Groundtruth";

    forward_velocity_plots.emplace_back(
        serial_robot_data_[id].odometry.interpolated);
    forward_velocity_plots.back().settings.title = "Interpolated";
    forward_velocity_plots.back().settings.linecolor = gnuplot::Colour::RED;

    plot(forward_velocity_plots, forward_velocity_axis);

    /* Angular Velcoty Plot */
    PlotList angular_velocity_plots;

    angular_velocity_plots.emplace_back(
        serial_robot_data_[id].odometry.groundtruth);

    angular_velocity_plots.back().settings.title = "Groundtruth";
    angular_velocity_plots.back().settings.y = ANGULAR_VELOCITY;

    angular_velocity_plots.emplace_back(
        serial_robot_data_[id].odometry.interpolated);

    angular_velocity_plots.back().settings.title = "Interpolated";
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

void Plotter::plot(const PlotList &plots,
                   const gnuplot::AxisSettings &axis_settings) {

  size_t total_plots = plots.size();

  /* Set the axis settings. */
  gnuplot_ << gnuplot::setAxisSettings(axis_settings);

  for (size_t i = 0; i < total_plots; ++i) {
    char identifier = 'A' + i;
    std::string dataset_name = "$data" + std::string(1, identifier);

    std::visit(
        [this, dataset_name](const auto &vec) {
          gnuplot_ << dataset_name + " << EOD\n";
          gnuplot_.send1d(vec);
          gnuplot_ << "EOD\n";
        },
        plots[i].dataset);
  }

  gnuplot_ << "plot";

  for (size_t i = 0; i < total_plots; ++i) {
    char identifier = 'A' + i;
    std::string dataset_name = "$data" + std::string(1, identifier);

    std::string plot_part =
        " " + dataset_name + gnuplot::setPlotSettings(plots[i].settings);

    gnuplot_ << plot_part;

    if (i < total_plots - 1) {
      gnuplot_ << ",";
    }
  }

  gnuplot_ << "\n";
}
} // namespace Data
