#include "Plotter.h"
#include "Landmark.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#ifdef REUSE
#include <filesystem>
#endif // REUSE

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

    /* Groundtruth State Data. */
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

    /* Binarise raw odometry. */
    const std::vector<Robot::Odometry> &raw_odometry =
        input_robot_data[id].raw.odometry;

    binary_robot_data_[id].odometry.raw = dataset_name_ + "_Robot_" +
                                          std::to_string(id + 1) + "_raw" +
                                          "_odometry";

    write_binary(binary_robot_data_[id].odometry.raw, raw_odometry);

    /* Binarise error odometry. */
    const std::vector<Robot::Odometry> &error_odometry =
        input_robot_data[id].error.odometry;

    binary_robot_data_[id].odometry.error = dataset_name_ + "_Robot_" +
                                            std::to_string(id + 1) + "_error" +
                                            "_odometry";

    write_binary(binary_robot_data_[id].odometry.error, error_odometry);
  }
}

void Plotter::binariseMeasurementData(unsigned short robot_id) {
  std::vector<Robot> &output_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    /* Synced measurements. */
    const std::vector<Robot::Measurement> &synced_measurements =
        output_robot_data[id].synced.measurements;

    std::string &synced_filename = binary_robot_data_[id].measurement.synced;

    synced_filename = dataset_name_ + "_Robot_" + std::to_string(id + 1) +
                      "_synced" + "_measurement";

    write_binary(synced_filename, synced_measurements);

    /* Groundtruth measurements. */
    const std::vector<Robot::Measurement> &groundtruth_measurements =
        output_robot_data[id].groundtruth.measurements;

    std::string &groundtruth_filename =
        binary_robot_data_[id].measurement.groundtruth;

    groundtruth_filename = dataset_name_ + "_Robot_" + std::to_string(id + 1) +
                           "_groundtruth" + "_measurement";

    write_binary(groundtruth_filename, groundtruth_measurements);

    /* Raw measurement */
    const std::vector<Robot::Measurement> &raw_measurements =
        output_robot_data[id].raw.measurements;

    std::string &raw_filename = binary_robot_data_[id].measurement.raw;

    raw_filename = dataset_name_ + "_Robot_" + std::to_string(id + 1) + "_raw" +
                   "_measurement";

    write_binary(raw_filename, raw_measurements);

    /* Error measurement */
    const std::vector<Robot::Measurement> &error_measurements =
        output_robot_data[id].error.measurements;

    std::string &error_filename = binary_robot_data_[id].measurement.error;

    error_filename = dataset_name_ + "_Robot_" + std::to_string(id + 1) +
                     "_error" + "_measurement";

    write_binary(error_filename, error_measurements);
  }
}

void Plotter::binariseMeasurementPDF(unsigned short robot_id,
                                     const double bin_size) {

  std::vector<Robot> &robots = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; id++) {
    std::unordered_map<int, double> range_bin_counts;
    std::unordered_map<int, double> bearing_bin_counts;

    /* Create PDFs. */
    double number_of_measurements = total_measurements_[id];

    for (const auto &measurement : robots[id].error.measurements) {
      for (unsigned short i = 0; i < measurement.subjects.size(); ++i) {

        int range_bin_index =
            static_cast<int>(std::floor(measurement.ranges[i] / bin_size));

        int bearing_bin_index =
            static_cast<int>(std::floor(measurement.bearings[i] / bin_size));

        range_bin_counts[range_bin_index] +=
            1.0 / (number_of_measurements * bin_size);

        bearing_bin_counts[bearing_bin_index] +=
            1.0 / (number_of_measurements * bin_size);
      }
    }

    std::string &range_filename = binary_robot_data_[id].range_pdf.filename;

    range_filename =
        dataset_name_ + "_Robot_" + std::to_string(id + 1) + "_range" + "_pdf";

    write_binary(range_filename, range_bin_counts, bin_size);

    std::string &bearing_filename = binary_robot_data_[id].bearing_pdf.filename;

    bearing_filename = dataset_name_ + "_Robot_" + std::to_string(id + 1) +
                       "_bearing" + "_pdf";

    write_binary(bearing_filename, bearing_bin_counts, bin_size);
  }
}

void Plotter::binariseOdometryPDF(unsigned short robot_id,
                                  const double bin_size) {
  std::vector<Robot> &robots = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; id++) {
    std::unordered_map<int, double> forward_velocity_bin_counts;
    std::unordered_map<int, double> angular_velocity_bin_counts;

    /* Create PDFs for forward and angular velocity. */
    for (const auto &odometry : robots[id].error.odometry) {
      int forward_velocity_bin_index =
          static_cast<int>(std::floor(odometry.forward_velocity / bin_size));

      int angular_velocity_bin_index =
          static_cast<int>(std::floor(odometry.angular_velocity / bin_size));

      /* NOTE: The bin count is actually the area contribution of the odometry
       * error for the given measurement. This means that the output is a
       * discretized pdf, where the sum of the area of all the bins should
       * equal 1. This is done for better visualisation when fitting a
       * Gaussian curve to the data. */
      forward_velocity_bin_counts[forward_velocity_bin_index] +=
          1.0 / (robots[id].error.odometry.size() * bin_size);

      angular_velocity_bin_counts[angular_velocity_bin_index] +=
          1.0 / (robots[id].error.odometry.size() * bin_size);
    }

    std::string &forward_velocity_filename =
        binary_robot_data_[id].forward_velocity_pdf.filename;

    forward_velocity_filename = dataset_name_ + "_Robot_" +
                                std::to_string(id + 1) + "_forward_velocity" +
                                "_pdf";

    write_binary(forward_velocity_filename, forward_velocity_bin_counts,
                 bin_size);

    std::string &angular_velocity_filename =
        binary_robot_data_[id].angular_velocity_pdf.filename;

    angular_velocity_filename = dataset_name_ + "_Robot_" +
                                std::to_string(id + 1) + "_angular_velocity" +
                                "_pdf";

    write_binary(angular_velocity_filename, angular_velocity_bin_counts,
                 bin_size);
  }
}

void Plotter::binariseRobotInferenceData(unsigned short robot_id) {

  std::vector<Robot> &output_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {

    /* Inference Data */
    const std::vector<Robot::State> &estimated_states =
        output_robot_data[id].synced.states;

    binary_robot_data_[id].pose.synced = dataset_name_ + "_Robot_" +
                                         std::to_string(id + 1) + "_estimated" +
                                         "_odometry";

    write_binary(binary_robot_data_[id].pose.synced, estimated_states);

    /* Error Data */
    const std::vector<Robot::State> &error_states =
        output_robot_data[id].error.states;

    binary_robot_data_[id].pose.error = dataset_name_ + "_Robot_" +
                                        std::to_string(id + 1) + "_error" +
                                        "_odometry";

    write_binary(binary_robot_data_[id].pose.error, error_states);
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
 * Plots the xy trajectory of the robots alongside the positions of the
 * landmarks.
 * @param robot_id Identifier of the robot whose data you want to plot. If the
 * value is 0, then all robots data is plotted.
 */
void Plotter::plotGroundruthTrajectory(std::initializer_list<PlotType> plots,
                                       unsigned short robot_id) {
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
 * Plots the groundtruth states of the robots.
 * @param robot_id The id of the robot whose states will be plotted.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotGroundruthStates(std::initializer_list<PlotType> plots,
                                   unsigned short robot_id) {
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

    gnuplot::AxisSettings x_position_axis;
    x_position_axis.x_label = "Time [s]";
    x_position_axis.y_label = "X Position [m]";

    gnuplot::AxisSettings y_position_axis;
    y_position_axis.x_label = "Time [s]";
    y_position_axis.y_label = "Y Position [m]";

    gnuplot::AxisSettings orientation_axis;
    orientation_axis.x_label = "Time [s]";
    orientation_axis.y_label = "Heading [rad]";

    PlotList x_plots;
    PlotList y_plots;
    PlotList orientation_plots;

    std::string binary_format = binary_robot_data_[id].pose.binary_format;

    for (const PlotType &plot : plots) {
      std::string plot_type;
      std::string plot_title;

      switch (plot) {
      case GROUNDTRUTH:
        plot_type = binary_robot_data_[id].pose.groundtruth;
        plot_title = "Groundtruth";
        break;

      case SYNCED:
        plot_type = binary_robot_data_[id].pose.synced;
        plot_title = "Synced";
        break;

      case RAW:
        plot_type = binary_robot_data_[id].pose.raw;
        plot_title = "Raw";
        break;

      case ERROR:
        plot_type = binary_robot_data_[id].pose.error;
        plot_title = "Error";
        break;

      default:
        break;
      }

      /* Plot X Position */
      x_plots.emplace_back(plot_type, binary_format);

      x_plots.back().settings.title = plot_title;
      x_plots.back().settings.x = TIME;
      x_plots.back().settings.y = X_POSITION;

      /* Plot Y Position */
      y_plots.emplace_back(plot_type, binary_format);

      y_plots.back().settings.title = plot_title;
      y_plots.back().settings.x = TIME;
      y_plots.back().settings.y = Y_POSITION;

      /* Plot Orientation */
      orientation_plots.emplace_back(plot_type, binary_format);

      orientation_plots.back().settings.title = plot_title;
      orientation_plots.back().settings.x = TIME;
      orientation_plots.back().settings.y = ORIENTATION;
    }

    plot(x_plots, x_position_axis);
    plot(y_plots, y_position_axis);
    plot(orientation_plots, orientation_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

/**
 *
 */
void Plotter::plotOdometry(std::initializer_list<PlotType> plots,
                           unsigned short robot_id) {
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

    gnuplot::AxisSettings angular_velocity_axis;

    angular_velocity_axis.y_label = "Angular Velocity [rad/s]";
    angular_velocity_axis.x_label = "Time [s]";

    /* Forward Velcoty Plot */
    PlotList forward_velocity_plots;

    /* Angular Velcoty Plot */
    PlotList angular_velocity_plots;

    std::string binary_format = binary_robot_data_[id].odometry.binary_format;

    for (const PlotType &plot : plots) {
      std::string plot_type;
      std::string plot_title;

      switch (plot) {
      case GROUNDTRUTH:
        plot_type = binary_robot_data_[id].odometry.groundtruth;
        plot_title = "Groundtruth";
        break;

      case SYNCED:
        plot_type = binary_robot_data_[id].odometry.synced;
        plot_title = "Synced";
        break;

      case RAW:
        plot_type = binary_robot_data_[id].odometry.raw;
        plot_title = "Raw";
        break;

      case ERROR:
        plot_type = binary_robot_data_[id].odometry.error;
        plot_title = "Error";
        break;

      default:
        break;
      }

      /* Range plot */
      forward_velocity_plots.emplace_back(plot_type, binary_format);

      forward_velocity_plots.back().settings.title = plot_title;
      forward_velocity_plots.back().settings.x = TIME;
      forward_velocity_plots.back().settings.y = FORWARD_VELOCITY;

      angular_velocity_plots.emplace_back(plot_type, binary_format);

      angular_velocity_plots.back().settings.title = plot_title;
      angular_velocity_plots.back().settings.x = TIME;
      angular_velocity_plots.back().settings.y = ANGULAR_VELOCITY;
    }

    plot(forward_velocity_plots, forward_velocity_axis);

    plot(angular_velocity_plots, angular_velocity_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

void Plotter::plotOdometryPDFs(unsigned short robot_id, const double bin_size) {
  if (robot_id > total_robots_ || robot_id <= 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseOdometryPDF(robot_id, bin_size);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  gnuplot::AxisSettings forward_velocity_axis;
  forward_velocity_axis.x_label = "Error [m/s]";
  forward_velocity_axis.y_label = "Probability Density [s/m]";

  gnuplot::AxisSettings angular_velocity_axis;
  angular_velocity_axis.x_label = "Error [rad/s]";
  angular_velocity_axis.y_label = "Probability Density [s/rad]";

  for (; id < end_point; ++id) {
    std::string title = "Odometry PDF";

    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::setMultiplot(2, 1);
    gnuplot_ << gnuplot::grid();

    PlotList forward_velocity_pdf;

    forward_velocity_pdf.emplace_back(
        binary_robot_data_[id].forward_velocity_pdf.filename,
        binary_robot_data_[id].forward_velocity_pdf.binary_format);
    forward_velocity_pdf.back().settings.x = 1;
    forward_velocity_pdf.back().settings.y = 3;
    forward_velocity_pdf.back().settings.box_width = 2;
    forward_velocity_pdf.back().settings.style = gnuplot::BOXES;

    PlotList angular_velocity_pdf;

    angular_velocity_pdf.emplace_back(
        binary_robot_data_[id].angular_velocity_pdf.filename,
        binary_robot_data_[id].angular_velocity_pdf.binary_format);
    angular_velocity_pdf.back().settings.x = 1;
    angular_velocity_pdf.back().settings.y = 3;
    angular_velocity_pdf.back().settings.box_width = 2;
    angular_velocity_pdf.back().settings.style = gnuplot::BOXES;

    plot(forward_velocity_pdf, forward_velocity_axis);
    plot(angular_velocity_pdf, angular_velocity_axis);
    gnuplot_.flush();
  }
}

void Plotter::plotMeasurements(std::initializer_list<PlotType> plots,
                               unsigned short robot_id) {
  if (robot_id > total_robots_ || robot_id <= 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseMeasurementData(robot_id);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Measurements";
    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::setMultiplot(2, 1);
    gnuplot_ << gnuplot::grid();

    /* Range Plot */
    PlotList range_plots;

    /* Angular Plot */
    PlotList bearing_plots;

    /* Set forward velocity axis labels */
    gnuplot::AxisSettings range_axis;
    range_axis.y_label = "Range [m]";
    range_axis.x_label = "Time [s]";

    gnuplot::AxisSettings bearing_axis;
    bearing_axis.y_label = "Bearing [rad]";
    bearing_axis.x_label = "Time [s]";

    std::string binary_format =
        binary_robot_data_[id].measurement.binary_format;

    for (const PlotType &plot : plots) {
      std::string plot_type;
      std::string plot_title;

      switch (plot) {
      case GROUNDTRUTH:
        plot_type = binary_robot_data_[id].measurement.groundtruth;
        plot_title = "Groundtruth";
        break;

      case SYNCED:
        plot_type = binary_robot_data_[id].measurement.synced;
        plot_title = "Synced";
        break;

      case RAW:
        plot_type = binary_robot_data_[id].measurement.raw;
        plot_title = "Raw";
        break;

      case ERROR:
        plot_type = binary_robot_data_[id].measurement.error;
        plot_title = "Error";
        break;
      default:
        break;
      }

      /* Range plot */
      range_plots.emplace_back(plot_type, binary_format);
      range_plots.back().settings.title = plot_title;
      range_plots.back().settings.x = TIME;
      range_plots.back().settings.y = RANGE;

      bearing_plots.emplace_back(plot_type, binary_format);
      bearing_plots.back().settings.title = plot_title;
      bearing_plots.back().settings.x = TIME;
      bearing_plots.back().settings.y = BEARING;
    }

    plot(range_plots, range_axis);
    plot(bearing_plots, bearing_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

void Plotter::plotMeasurementPDFs(unsigned short robot_id,
                                  const double bin_size) {
  if (robot_id > total_robots_ || robot_id <= 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseMeasurementPDF(robot_id, bin_size);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  gnuplot::AxisSettings range_axis;
  range_axis.x_label = "Error [m]";
  range_axis.y_label = "Probability Density [1/m]";

  gnuplot::AxisSettings bearing_axis;
  bearing_axis.x_label = "Error [rad]";
  bearing_axis.y_label = "Probability Density [1/rad]";

  for (; id < end_point; ++id) {
    std::string title = "Measurement PDF";

    gnuplot_ << gnuplot::setTitle(title);

    gnuplot_ << gnuplot::setTerminal(id);
    gnuplot_ << gnuplot::setMultiplot(2, 1);
    gnuplot_ << gnuplot::grid();

    PlotList range_pdf;

    range_pdf.emplace_back(binary_robot_data_[id].range_pdf.filename,
                           binary_robot_data_[id].range_pdf.binary_format);
    range_pdf.back().settings.x = 1;
    range_pdf.back().settings.y = 3;
    range_pdf.back().settings.box_width = 2;
    range_pdf.back().settings.style = gnuplot::BOXES;

    PlotList bearing_pdf;

    bearing_pdf.emplace_back(binary_robot_data_[id].bearing_pdf.filename,
                             binary_robot_data_[id].bearing_pdf.binary_format);
    bearing_pdf.back().settings.x = 1;
    bearing_pdf.back().settings.y = 3;
    bearing_pdf.back().settings.box_width = 2;
    bearing_pdf.back().settings.style = gnuplot::BOXES;

    plot(range_pdf, range_axis);
    plot(bearing_pdf, bearing_axis);
    gnuplot_.flush();
  }
}

/**
 * Writes binary file for odometry data.
 * @param filename the name of the output binary file.
 * @param odometry_data Vector of measurements.
 */
void Plotter::write_binary(std::string &filename,
                           const std::vector<Robot::Odometry> &odometry_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

#ifdef REUSE
  if (std::filesystem::exists(filename)) {
    return;
  }
#endif // REUSE

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

/**
 * Writes binary file for pose data.
 * @param filename the name of the output binary file.
 * @param state_data Vector of measurements.
 */
void Plotter::write_binary(std::string &filename,
                           const std::vector<Robot::State> &state_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

#ifdef REUSE
  if (std::filesystem::exists(filename)) {
    return;
  }
#endif // REUSE

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

/**
 * Writes binary file for measurement data.
 * @param filename the name of the output binary file.
 * @param measurement_data Vector of measurements.
 */
void Plotter::write_binary(
    std::string &filename,
    const std::vector<Robot::Measurement> &measurement_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

#ifdef REUSE
  if (std::filesystem::exists(filename)) {
    return;
  }
#endif // REUSE

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  for (auto const &row : measurement_data) {
    for (unsigned short i = 0; i < row.subjects.size(); ++i) {
      fout.write(reinterpret_cast<const char *>(&row.time), sizeof(double));
      fout.write(reinterpret_cast<const char *>(&row.subjects[i]),
                 sizeof(unsigned short));
      fout.write(reinterpret_cast<const char *>(&row.ranges[i]),
                 sizeof(double));
      fout.write(reinterpret_cast<const char *>(&row.bearings[i]),
                 sizeof(double));
      if (!fout) {
        throw std::runtime_error("Failed to write time data");
      }
    }
  }

  fout.close();
}

/**
 * Writes binary file for landmark data.
 * @param filename the name of the output binary file.
 * @param landmark_data Vector of landmarks.
 */
void Plotter::write_binary(std::string &filename,
                           const std::vector<Landmark> &landmark_data) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

#ifdef REUSE
  if (std::filesystem::exists(filename)) {
    return;
  }
#endif // REUSE

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

void Plotter::write_binary(std::string &filename,
                           const std::unordered_map<int, double> &pdf_data,
                           const double &bin_size) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  for (const auto &[bin_index, count] : pdf_data) {
    const double bin_start = bin_index * bin_size;
    const double bin_end = bin_start + bin_size;
    const double bin_centre = (bin_start + bin_end) / 2;

    fout.write(reinterpret_cast<const char *>(&bin_centre), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&bin_size), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&count), sizeof(double));
  }

  fout.close();
}

/**
 * Sends plot commands to gnuplot.
 */
void Plotter::plot(const PlotList &plots,
                   const gnuplot::AxisSettings &axis_settings) {

  size_t total_plots = plots.size();

  /* Set the axis settings. */

  gnuplot_ << gnuplot::setAxisSettings(axis_settings);

  std::ostringstream plot_command;

  plot_command << "plot ";

  for (size_t i = 0; i < total_plots; ++i) {
    assert(plots[i].binary_name != "" && "Dataset name not set.");

    plot_command << "'" << plots[i].binary_name << "'" << " binary "
                 << "format='" << plots[i].binary_format << "'"
                 << gnuplot::setPlotSettings(plots[i].settings);

    if (i < total_plots - 1) {
      plot_command << ",";
    }
  }

  plot_command << "\n";

  gnuplot_ << plot_command.str();
  /* Close all the files. */

  std::cout << plot_command.str() << std::endl;
#ifdef DEBUG
  std::cout << plot_command.str() << std::endl;
#endif // DEBUG
}

} // namespace Data
