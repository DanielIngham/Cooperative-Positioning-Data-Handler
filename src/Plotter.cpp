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
        input_robot_data[id].groundtruth.odometry;

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

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << "set grid\n";
    gnuplot_ << gnuplot::setMultiplot(3, 1, title);

    gnuplot_ << "$groundtruth_pose << EOD\n";
    gnuplot_.send1d(serial_robot_data_[id].pose.groundtruth);
    gnuplot_ << "EOD\n";

    /* Let style */
    gnuplot_ << "set style data points\n";
    gnuplot_ << "set pointsize 0.1\n";

    gnuplot_ << "plot $groundtruth_pose using 1:2 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_ << "plot $groundtruth_pose using 1:3 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_ << "plot $groundtruth_pose using 1:4 title 'Interpolated' lc rgb "
                "'purple'\n";

    gnuplot_ << gnuplot::unsetMultiplot();

    gnuplot_.flush();
  }
}

/**
 * Plots the xy trajectory of the robots alongside the positions of the
 * landmarks.
 */
void Plotter::plotGroundruthTrajectory(unsigned short robot_id) {

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Groundtruth";

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << "set grid\n";

    gnuplot_ << "$groundtruth_pose << EOD\n";
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

void Plotter::plotOdometry(unsigned short robot_id) {
  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Groundtruth";

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::setMultiplot(2, 1);
    gnuplot_ << gnuplot::setGrid();

    gnuplot::PlotSettings forward_velocity;

    plot({serial_robot_data_[id].odometry.groundtruth,
          serial_robot_data_[id].odometry.interpolated},
         {forward_velocity, forward_velocity});

    gnuplot::PlotSettings angular_velocity;
    angular_velocity.y = 3;

    plot({serial_robot_data_[id].odometry.groundtruth,
          serial_robot_data_[id].odometry.interpolated},
         {angular_velocity, angular_velocity});

    // gnuplot_ << "plot $interpolated_odometry using 1:2 with points pointsize
    // "
    //             "1.0 pointtype 6 title 'Interpolated',"
    //          << " $groundtruth_odometry using 1:2 with points pointsize 1.0 "
    //             "pointtype 6 title 'Interpolated'\n";

    // gnuplot_
    //     << "plot $interpolated_odometry using 1:3 with points pointsize 1.0 "
    //        "pointtype 6 title 'Interpolated',"
    //     << " $groundtruth_odometry using 1:3 with points pointsize 1.0 "
    //        "pointtype 6 title 'Interpolated'\n";

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

#if 0

void Plotter::plot(const std::vector<PlotData> &datasets,
                   const std::vector<gnuplot::PlotSettings> &plot_settings) {

  size_t total_plots = datasets.size();
  std::string command = "";

  for (size_t i = 0; i < total_plots; ++i) {

    char identifier = 'A' + i;
    std::string dataset_name = &"$data"[identifier];

    std::visit(
        [i, this, dataset_name, plot_settings](const auto &vec) {
          using VecType = std::decay_t<decltype(vec)>;

          gnuplot_ << dataset_name + " << EOD\n";
          gnuplot_.send1d(vec);
          gnuplot_ << "EOD\n";
        },
        datasets[i]);

    if (i == 0) {
      gnuplot_ << "plot";
      command += "plot";
    }

    gnuplot_ << " " + dataset_name + gnuplot::command(plot_settings[i]);
    command += " " + dataset_name + gnuplot::command(plot_settings[i]);

    if (i == total_plots - 1) {
      gnuplot_ << '\n';
      command += '\n';
    } else {
      gnuplot_ << ',';
      command += ',';
    }
  }
  std::cout << command << std::endl;
}
#endif // 0
void Plotter::plot(const std::vector<PlotData> &datasets,
                   const std::vector<gnuplot::PlotSettings> &plot_settings) {

  size_t total_plots = datasets.size();
  std::string command = "";

  // First, define all data blocks
  for (size_t i = 0; i < total_plots; ++i) {
    char identifier = 'A' + i;
    std::string dataset_name = "$data" + std::string(1, identifier);

    std::visit(
        [this, dataset_name](const auto &vec) {
          gnuplot_ << dataset_name + " << EOD\n";
          gnuplot_.send1d(vec);
          gnuplot_ << "EOD\n";
        },
        datasets[i]);
  }

  // Then create the plot command
  gnuplot_ << "plot";
  command += "plot";

  for (size_t i = 0; i < total_plots; ++i) {
    char identifier = 'A' + i;
    std::string dataset_name = "$data" + std::string(1, identifier);

    std::string plot_part =
        " " + dataset_name + gnuplot::command(plot_settings[i]);

    gnuplot_ << plot_part;
    command += plot_part;

    if (i < total_plots - 1) {
      gnuplot_ << ",";
      command += ",";
    }
  }

  gnuplot_ << "\n";
  command += "\n";

  std::cout << command << std::endl;
}
} // namespace Data
