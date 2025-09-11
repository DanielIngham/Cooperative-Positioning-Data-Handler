#include "Plotter.h"
#include "Landmark.h"
#include "Robot.h"

#include <algorithm>
#include <boost/current_function.hpp>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#ifdef REUSE
#include <filesystem>
#endif // REUSE

namespace Data {
unsigned short Plotter::terminal_number_{};

/**
 * Constructor for instance of the Plotter which sets its fields using the data
 * extracted by the Handler.
 * @param data Instance of the Handler class that has been assigned a dataset.
 * @note NOTE: At this state, the plotter requires the data Handler to have its
 * dataset set before creating an instance of the plotter.
 */
Plotter::Plotter(Handler &data)
    : data_(data), dataset_name_(data_.getDatasetName()),
      data_points_(data_.getNumberOfSyncedDatapoints()),
      total_measurements_(data_.getNumberOfSyncedMeasurements()),
      total_robots_(data_.getNumberOfRobots()),
      total_landmarks_(data_.getNumberOfLandmarks()) {

  assert(dataset_name_ != "" &&
         "Data set not set. Set the dataset in the Handler before attempting "
         "to create an instance of the plotter.");

  binary_robot_data_.resize(total_robots_);

  data_extraction_directory_ = data_.getDataExtractionDirectory();
  data_infernce_directory_ = data_.getDataInferenceDirectory();
  animation_directory_ = data_extraction_directory_ + "/animation/";
}

/**
 * Default destructor.
 */
Plotter::~Plotter() {}

/**
 * Sets the terminal to be used by gnuplot.
 * @param terminal gnuplot terminal structure containing all gnuplot terminal
 * settings.
 */
void Plotter::setTerminal(gnuplot::TerminalSettings terminal) {
  terminal_ = terminal;
}

/**
 * Converts the robots Pose data extracted by the Handler into the binary format
 * and saves it to a binary file in the termporary directory that gnuplot can
 * use for plotting.
 * @param plots The types of plots whose data should be binarised.
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @note If the ID number is 0, all robot plots will be shown.
 */
void Plotter::binariseRobotPoseData(std::initializer_list<PlotType> plots,
                                    const unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Robot ID provided is not valid: " +
                             std::to_string(robot_id));
  }

  std::vector<Robot> &input_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    for (const PlotType &plot : plots) {
      const std::vector<Robot::State> *poses;
      std::string *filename;
      switch (plot) {

      case GROUNDTRUTH:
        poses = &input_robot_data[id].groundtruth.states;
        filename = &binary_robot_data_[id].pose.groundtruth;
        *filename = "Groundtruth_";
        break;

      case RAW:
        poses = &input_robot_data[id].raw.states;
        filename = &binary_robot_data_[id].pose.raw;
        *filename = "Raw_";
        break;

        /* NOTE: The synced data corresponds to the inference data. */
      case SYNCED:
        poses = &input_robot_data[id].synced.states;
        filename = &binary_robot_data_[id].pose.synced;
        *filename = "Synced_";
        break;

      case ERROR:
        poses = &input_robot_data[id].error.states;
        filename = &binary_robot_data_[id].pose.error;
        *filename = "Error_";
        break;

      case ABSOLUTE_ERROR:
        poses = &input_robot_data[id].absolute_state_error;
        filename = &binary_robot_data_[id].absolute_pose_error;
        *filename = "Absolute_Error_";
        break;

      default:
        throw std::runtime_error("Plot type not known");
      }

      *filename +=
          dataset_name_ + "_Robot_" + std::to_string(id + 1) + +"_poses";

      write_binary(*filename, *poses);
    }
  }
}

/**
 * Coverts robot odometry sensor values to temporary binary files for gnuplot.
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @note If the ID number is 0, all robot plots will be shown.
 */
void Plotter::binariseOdometryData(std::initializer_list<PlotType> plots,
                                   const unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Robot ID provided is not valid: " +
                             std::to_string(robot_id));
  }
  std::vector<Robot> &input_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    const std::vector<Robot::Odometry> *odometry;
    std::string *filename;
    for (const PlotType &plot : plots) {

      switch (plot) {
      case GROUNDTRUTH:
        odometry = &input_robot_data[id].groundtruth.odometry;
        filename = &binary_robot_data_[id].odometry.groundtruth;
        *filename = "Groundtruth_";
        break;

      case RAW:
        odometry = &input_robot_data[id].raw.odometry;
        filename = &binary_robot_data_[id].odometry.raw;
        *filename = "Raw_";
        break;

      case SYNCED:
        odometry = &input_robot_data[id].synced.odometry;
        filename = &binary_robot_data_[id].odometry.synced;
        *filename = "Synced_";
        break;

      case ERROR:
        odometry = &input_robot_data[id].error.odometry;
        filename = &binary_robot_data_[id].odometry.error;
        *filename = "Error_";
        break;

      default:
        throw std::runtime_error("Plot type unknown");
      }

      *filename +=
          dataset_name_ + "_Robot_" + std::to_string(id + 1) + +"_odometry";

      write_binary(*filename, *odometry);
    }
  }
}

/**
 * Converts the robot measurement data into binary files for gnuplot.
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @note If the ID number is 0, all robot plots will be shown.
 */
void Plotter::binariseMeasurementData(std::initializer_list<PlotType> plots,
                                      unsigned short robot_id) {

  std::vector<Robot> &output_robot_data = data_.getRobots();

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {

    for (const PlotType &plot : plots) {

      const std::vector<Robot::Measurement> *measurements;
      std::string *filename;

      switch (plot) {
      case GROUNDTRUTH:
        measurements = &output_robot_data[id].groundtruth.measurements;
        filename = &binary_robot_data_[id].measurement.groundtruth;
        *filename = "Groundtruth_";
        break;

      case RAW:
        measurements = &output_robot_data[id].raw.measurements;
        filename = &binary_robot_data_[id].measurement.raw;
        *filename = "Raw_";
        break;

      case SYNCED:
        measurements = &output_robot_data[id].synced.measurements;
        filename = &binary_robot_data_[id].measurement.synced;
        *filename = "Synced_";
        break;

      case ERROR:
        measurements = &output_robot_data[id].error.measurements;
        filename = &binary_robot_data_[id].measurement.error;
        *filename = "Error_";
        break;

      default:

        break;
      }

      *filename +=
          dataset_name_ + "_Robot_" + std::to_string(id + 1) + "_measurement";

      write_binary(*filename, *measurements);
    }
  }
}

void Plotter::binariseMeasurementVectors(std::initializer_list<PlotType> plots,
                                         const unsigned short robot_id) {

  const std::vector<Robot> &output_robot_data{data_.getRobots()};

  unsigned short end_point{(robot_id == 0) ? total_robots_ : robot_id},
      id{static_cast<unsigned short>((robot_id == 0) ? 0 : robot_id - 1)};

  const size_t total_datapoints{data_.getNumberOfSyncedDatapoints()};

  for (; id < end_point; ++id) {

    /* Prepend temporary directory to filename. */
    std::string *filename = &binary_robot_data_[id].measurement_vector.filename;

    *filename =
        "/tmp/Robot_" + std::to_string(id) + "measurment_vectors" + ".bin";

    std::ofstream fout(*filename, std::ios::binary);

    if (!fout) {
      throw std::runtime_error("Could not open temporary file");
    }

    /* Loop through each measurement. */
    for (const Robot::Measurement &measurement :
         output_robot_data[id].groundtruth.measurements) {

      const Robot::State *pose;

      /* Find the index in the ground truth that corresponds to the measurement
       * time. */
      for (size_t k{}; k < total_datapoints; ++k) {
        pose = &output_robot_data[id].groundtruth.states[k];

        /* Round to 4 decimal places */
        static constexpr double decimal_threshold{1e-4};
        if (std::abs(pose->time - measurement.time) < decimal_threshold) {
          break;
        }
      }

      for (unsigned int i{}; i < measurement.subjects.size(); i++) {

        double y{measurement.ranges[i] *
                 std::sin(measurement.bearings[i] + pose->orientation)};
        double x{measurement.ranges[i] *
                 std::cos(measurement.bearings[i] + pose->orientation)};

        /* Convert odometry data to binary. */
        fout.write(reinterpret_cast<const char *>(&pose->x), sizeof(double));
        fout.write(reinterpret_cast<const char *>(&pose->y), sizeof(double));
        fout.write(reinterpret_cast<const char *>(&x), sizeof(double));
        fout.write(reinterpret_cast<const char *>(&y), sizeof(double));
      }
    }
    fout.close();
  }
}

/**
 * Groups the robots measurement error for its range and bearing sensing, and
 * creates a quasi PDF (scaled PMF).
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @param bin_size Size of the bins in which the error values are grouped.
 * @note If the ID number is 0, all robot plots will be shown.
 */
void Plotter::binariseMeasurementPDF(unsigned short robot_id,
                                     const double bin_size) {

  std::vector<Robot> &robots = data_.getRobots();

  unsigned short end_point{(robot_id == 0) ? total_robots_ : robot_id},
      id{static_cast<unsigned short>((robot_id == 0) ? 0 : robot_id - 1)};

  for (; id < end_point; id++) {
    std::unordered_map<int, double> range_bin_counts;
    std::unordered_map<int, double> bearing_bin_counts;

    /* Create PDFs. */
    double number_of_measurements = total_measurements_[id];

    for (const auto &measurement : robots[id].error.measurements) {
      for (unsigned short i{}; i < measurement.subjects.size(); ++i) {

        int range_bin_index{
            static_cast<int>(std::floor(measurement.ranges[i] / bin_size))};

        int bearing_bin_index{
            static_cast<int>(std::floor(measurement.bearings[i] / bin_size))};

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

/**
 * Groups the robots odometry error for its range and bearing sensing, and
 * creates a quasi PDF (scaled PMF).
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @param bin_size Size of the bins in which the error values are grouped.
 * @note If the ID number is 0, all robot plots will be shown.
 */
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

/**
 * Converts the landmark position data structure into a binary file for gnuplt.
 */
void Plotter::binariseLandmarkData() {

  const std::vector<Landmark> &input_landmark_data = data_.getLandmarks();

  binary_landmark_data_.filename = dataset_name_ + "_Landmarks";

  write_binary(binary_landmark_data_.filename, input_landmark_data);
}

/**
 * Creates a gif of the optimisation parameter values after each iteration.
 * @param plots the types of plots the should be created from the parameter
 * values.
 * TODO: Add ability to select different robots based on ID.
 */
void Plotter::inferenceErrorAnimation(std::initializer_list<PlotType> plots) {
  static const unsigned short first_iteration{};

  const unsigned short rows{3U}, columns{1U};

  for (const auto &plot : plots) {
    gnuplot_ << gnuplot::grid();

    terminal_ = {
        .type = gnuplot::GIF,
        .number = ++terminal_number_,
    };

    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string output_file{animation_directory_};

    switch (plot) {
    case SYNCED:
      output_file += "synced_iterations";
      break;

    case ERROR:
      output_file += "error_iterations";
      break;

    case ABSOLUTE_ERROR:
      output_file += "absolute_error_iterations";
      break;

    default:
      throw std::runtime_error("The plot type provide is not accepted by the "
                               "inference animation function.");
    }

    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    for (unsigned short i{}; i < total_inference_iterations_; ++i) {
      gnuplot_ << gnuplot::setMultiplot(rows, columns);
      inferenceIterationsPlotter(plot, {first_iteration, i});
      gnuplot_ << gnuplot::unsetMultiplot();
    }
  }
}

void Plotter::trajectoryAnimation(const unsigned short robot_id) {
  gnuplot_ << gnuplot::grid();

  terminal_ = {
      .type = gnuplot::GIF,
      .number = ++terminal_number_,
  };

  gnuplot_ << gnuplot::setTerminal(terminal_);

  std::string output_filename{animation_directory_ + "trajectory_animation"};
  gnuplot_ << gnuplot::setOutput(output_filename, terminal_);

  binariseLandmarkData();
  binariseRobotPoseData({GROUNDTRUTH});

  static const unsigned short first_iteration{};
  for (unsigned short i{}; i < total_inference_iterations_; ++i) {
    trajectoryIterationsPlotter(robot_id, {first_iteration, i});
  }
}

/**
 * Plots the xy trajectory of the robots alongside the positions of the
 * landmarks.
 * @param plots List containing the type of data extracted/calculated from the
 * dataset.
 * @param robot_id Identifier of the robot whose data you want to plot. If the
 * value is 0, then all robots data is plotted.
 */
void Plotter::plotTrajectory(std::initializer_list<PlotType> plots,
                             unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseLandmarkData();
  binariseRobotPoseData(plots, robot_id);

  unsigned short end_point{(robot_id == 0) ? total_robots_ : robot_id};

  unsigned short id{
      static_cast<unsigned short>((robot_id == 0) ? 0 : robot_id - 1)};

  for (; id < end_point; ++id) {

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string title = "Robot " + std::to_string(id + 1) + " Trajectory";
    std::string output_file = data_extraction_directory_ + title;

    gnuplot_ << gnuplot::setOutput(output_file, terminal_);
    gnuplot_ << gnuplot::grid();

    gnuplot::AxisSettings axis{
        .title = title,
        .x_label = "x position [m]",
        .y_label = "y position [m]",
    };

    PlotList plot_list;

    plot_list.emplace_back(binary_landmark_data_.filename,
                           LandmarkData::binary_format());
    plot_list.back().settings = {
        .key_label = "Landmarks",
        .style = gnuplot::PlotStyle::POINTS,
    };

    for (const auto &input_plot : plots) {
      /* Create the plot for the trajectory of the robots. */
      std::string plot_file{};
      std::string key{};

      switch (input_plot) {
      case SYNCED:
        plot_file = binary_robot_data_[id].pose.groundtruth;
        key = "Synced trajectory";
        break;
      case GROUNDTRUTH:
        plot_file = binary_robot_data_[id].pose.groundtruth;
        key = "Groundtruth trajectory";
        break;
      default:
        throw std::runtime_error("Plot type not accepted for trajectory plot.");
      }

      plot_list.emplace_back(plot_file,
                             binary_robot_data_[id].pose.binary_format());
      plot_list.back().settings = {
          .key_label = key,
          .x = X_POSITION,
          .y = Y_POSITION,
          .style = gnuplot::PlotStyle::LINES,
      };
    }

    plot(plot_list, axis);

    gnuplot_.flush();
  }
}

void Plotter::plotMeasurementsVector(unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseLandmarkData();
  binariseRobotPoseData({GROUNDTRUTH}, robot_id);
  binariseMeasurementVectors({GROUNDTRUTH}, robot_id);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {

    std::string title{"Robot " + std::to_string(id + 1) +
                      " Measurement Vectors"};

    gnuplot_ << gnuplot::setTitle(title);

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string output_file = data_extraction_directory_ + title;
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);
    gnuplot_ << gnuplot::grid();

    PlotList plots;

    plots.emplace_back(binary_robot_data_[id].measurement_vector.filename,
                       RobotData::MeasurementVector::binary_format);
    plots.back().settings = {
        .key_label = "Measurement Vectors",
        .style = gnuplot::PlotStyle::VECTORS,
        .linecolor = gnuplot::Colour::LIGHT_RED,
    };

    plots.emplace_back(binary_landmark_data_.filename,
                       LandmarkData::binary_format());
    plots.back().settings = {
        .key_label = "Landmarks",
        .style = gnuplot::PlotStyle::POINTS,
        .pointtype = gnuplot::PointType::FILLED_PENTAGON,
        .pointsize = 4,
        .linecolor = gnuplot::Colour::BLACK,
    };

    plots.emplace_back(binary_robot_data_[id].pose.groundtruth,
                       binary_robot_data_[id].pose.binary_format());
    plots.back().settings = {
        .x = X_POSITION,
        .y = Y_POSITION,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::Colour::BLACK,
        .linewidth = 2,
    };

    /* NOTE: the first two coordinates of the measurement vector correspond to
     * the instances of the groundtruth trajectory where a measurement was
     * taken.*/
    plots.emplace_back(binary_robot_data_[id].measurement_vector.filename,
                       binary_robot_data_[id].measurement_vector.binary_format);
    plots.back().settings = {
        .key_label = "Trajectory",
    };

    gnuplot::AxisSettings axis{
        .x_label = "x position [m]",
        .y_label = "y position [m]",
    };

    plot(plots, axis);

    gnuplot_.flush();
  }
}

/**
 * Plots the groundtruth states (poses) of the robots.
 * @param plots List containing the type of data extracted/calculated from the
 * dataset.
 * @param robot_id The id of the robot whose states will be plotted.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotPoses(std::initializer_list<PlotType> plots,
                        unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0U) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseRobotPoseData(plots, robot_id);

  const unsigned short end_point = (robot_id == 0U) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0U) ? 0U : robot_id - 1U;

  std::string title = "Robot " + std::to_string(id + 1U) + " Groundtruth";
  gnuplot_ << gnuplot::setTitle(title);

  terminal_.number = ++terminal_number_;
  gnuplot_ << gnuplot::setTerminal(terminal_);

  std::string output_file = data_extraction_directory_ + title;
  gnuplot_ << gnuplot::setOutput(output_file, terminal_);

  gnuplot_ << gnuplot::grid();

  const unsigned short rows{3U}, columns{1U};
  gnuplot_ << gnuplot::setMultiplot(rows, columns);

  gnuplot::AxisSettings x_position_axis{
      .x_label = "Time [s]",
      .y_label = "X Position [m]",
  };

  gnuplot::AxisSettings y_position_axis{
      .x_label = "Time [s]",
      .y_label = "Y Position [m]",
  };

  gnuplot::AxisSettings orientation_axis{
      .x_label = "Time [s]",
      .y_label = "Heading [rad]",
  };

  PlotList x_plots, y_plots, orientation_plots;

  for (; id < end_point; ++id) {

    std::string binary_format = binary_robot_data_[id].pose.binary_format();

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

      case ABSOLUTE_ERROR:
        plot_type = binary_robot_data_[id].absolute_pose_error;
        plot_title = "Absolute Error";
        break;

      default:
        break;
      }

      static constexpr double point_size{0.5};

      x_plots.emplace_back(plot_type, binary_format);
      x_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = X_POSITION,
          .style = gnuplot::LINESPOINTS,
          .pointsize = point_size,
      };

      y_plots.emplace_back(plot_type, binary_format);
      y_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = Y_POSITION,
          .style = gnuplot::LINESPOINTS,
          .pointsize = point_size,
      };

      orientation_plots.emplace_back(plot_type, binary_format);

      orientation_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = HEADING,
          .style = gnuplot::LINESPOINTS,
          .pointsize = point_size,
      };
    }
  }

  plot(x_plots, x_position_axis);
  plot(y_plots, y_position_axis);
  plot(orientation_plots, orientation_axis);

  gnuplot_ << gnuplot::unsetMultiplot();
  gnuplot_.flush();
}

/**
 * Plots the odometry inputs that each vehicle recieved through a run in the
 * dataset.
 * @param plots List containing the type of data extracted/calculated from the
 * dataset.
 * @param robot_id The id of the robot whose states will be plotted.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotOdometry(std::initializer_list<PlotType> plots,
                           unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseOdometryData(plots, robot_id);

  const unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Odometry";

    gnuplot_ << gnuplot::setTitle(title);

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string output_file{data_extraction_directory_ + title};
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    const unsigned short rows{2U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);
    gnuplot_ << gnuplot::grid();

    /* Set forward velocity axis labels */
    gnuplot::AxisSettings forward_velocity_axis{
        .x_label = "Time [s]",
        .y_label = "Forward Velocity [m/s]",
    };

    gnuplot::AxisSettings angular_velocity_axis{
        .x_label = "Time [s]",
        .y_label = "Angular Velocity [rad/s]",
    };

    PlotList forward_velocity_plots, angular_velocity_plots;

    std::string binary_format = binary_robot_data_[id].odometry.binary_format();

    for (const PlotType &plot : plots) {
      std::string plot_type, plot_title;

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

      forward_velocity_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = FORWARD_VELOCITY,
      };

      angular_velocity_plots.emplace_back(plot_type, binary_format);

      angular_velocity_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = ANGULAR_VELOCITY,
      };
    }

    plot(forward_velocity_plots, forward_velocity_axis);

    plot(angular_velocity_plots, angular_velocity_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

/**
 * Plot the Probability Density Function (Scaled Pobability Mass Function (PMF))
 * for error of the odometry inputs.
 * @param robot_id The id of the robot whose states will be plotted.
 * @param bin_size Size of the bins in which the error values are grouped.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotOdometryPDFs(unsigned short robot_id, const double bin_size) {

  /* Check that the provided ID is within bounds. */
  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  std::vector<Robot> robots = data_.getRobots();

  binariseOdometryPDF(robot_id, bin_size);

  const unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  gnuplot::AxisSettings forward_velocity_axis{
      .title = "Forward Velocity Probability Distribution",
      .x_label = "Error [m/s]",
      .y_label = "Probability Density [s/m]",
  };

  gnuplot::AxisSettings angular_velocity_axis{
      .title = "Angular Velocity Noise Probability Distribution",
      .x_label = "Error [rad/s]",
      .y_label = "Probability Density [s/rad]",
  };

  for (; id < end_point; ++id) {
    std::string title = "Odometry PDF";

    gnuplot_ << gnuplot::setTitle(title);

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string output_file = data_extraction_directory_ + title;
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    const unsigned short rows{2U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);
    gnuplot_ << gnuplot::grid();

    PlotList forward_velocity_pdf;

    forward_velocity_pdf.emplace_back(
        binary_robot_data_[id].forward_velocity_pdf.filename,
        binary_robot_data_[id].forward_velocity_pdf.binary_format);

    forward_velocity_pdf.back().settings = {
        .key_label = "Scaled PMF",
        .x = 1,
        .y = 3,
        .box_width = 2,
        .style = gnuplot::BOXES,
    };

    double sigma{std::sqrt(robots[id].forward_velocity_error.variance)};
    double mu{robots[id].forward_velocity_error.mean};

    std::ostringstream forward_velocity_gaussian_plot;
    forward_velocity_gaussian_plot
        << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
        << "* exp(- " << "( x - " << mu << ")**2 / (2 *" << std::pow(sigma, 2)
        << "))";

    forward_velocity_pdf.emplace_back(forward_velocity_gaussian_plot.str());

    forward_velocity_pdf.back().settings = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    PlotList angular_velocity_pdf;

    angular_velocity_pdf.emplace_back(
        binary_robot_data_[id].angular_velocity_pdf.filename,
        binary_robot_data_[id].angular_velocity_pdf.binary_format);

    angular_velocity_pdf.back().settings = {
        .key_label = "Scaled PMF",
        .x = 1,
        .y = 3,
        .box_width = 2,
        .style = gnuplot::BOXES,
    };

    sigma = std::sqrt(robots[id].angular_velocity_error.variance);
    mu = robots[id].angular_velocity_error.mean;

    std::ostringstream angular_velocity_gaussian_plot;
    angular_velocity_gaussian_plot
        << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
        << "* exp(- " << "( x - " << mu << ")**2 / (2 *" << std::pow(sigma, 2)
        << "))";

    angular_velocity_pdf.emplace_back(angular_velocity_gaussian_plot.str());
    angular_velocity_pdf.back().settings = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    plot(forward_velocity_pdf, forward_velocity_axis);
    plot(angular_velocity_pdf, angular_velocity_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
} // namespace Data

/**
 * Plots the range and bearing measurements that each vehicle recieved through a
 * run in the dataset.
 * @param plots List containing the type of data extracted/calculated from the
 * dataset.
 * @param robot_id The id of the robot whose states will be plotted.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotMeasurements(std::initializer_list<PlotType> plots,
                               unsigned short robot_id) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseMeasurementData(plots, robot_id);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  for (; id < end_point; ++id) {
    std::string title = "Robot " + std::to_string(id + 1) + " Measurements";
    gnuplot_ << gnuplot::setTitle(title);

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string output_file = data_extraction_directory_ + title;
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    static constexpr unsigned short rows{2U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);

    gnuplot_ << gnuplot::grid();

    PlotList range_plots, bearing_plots;

    /* Set forward velocity axis labels */
    gnuplot::AxisSettings range_axis{
        .x_label = "Time [s]",
        .y_label = "Range [m]",
    };

    gnuplot::AxisSettings bearing_axis{
        .x_label = "Time [s]",
        .y_label = "Bearing [rad]",
    };

    std::string binary_format =
        binary_robot_data_[id].measurement.binary_format();

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
      range_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = RANGE,
      };

      bearing_plots.emplace_back(plot_type, binary_format);
      bearing_plots.back().settings = {
          .key_label = plot_title,
          .x = TIME,
          .y = BEARING,
      };
    }

    plot(range_plots, range_axis);
    plot(bearing_plots, bearing_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

/**
 * Plot the Probability Density Function (Scaled Pobability Mass Function (PMF))
 * for error of the range and bearing measurements.
 * @param robot_id The id of the robot whose states will be plotted.
 * @param bin_size Size of the bins in which the error values are grouped.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotMeasurementPDFs(unsigned short robot_id,
                                  const double bin_size) {

  if (robot_id > total_robots_ || robot_id < 0) {
    throw std::runtime_error("Invalid robot id: " + std::to_string(robot_id));
  }

  binariseMeasurementPDF(robot_id, bin_size);

  unsigned short end_point = (robot_id == 0) ? total_robots_ : robot_id;

  unsigned short id = (robot_id == 0) ? 0 : robot_id - 1;

  gnuplot::AxisSettings range_axis{
      .title = "Range Noise Probability Distribution",
      .x_label = "Error [m]",
      .y_label = "Probability Density [1/m]",
  };

  gnuplot::AxisSettings bearing_axis{
      .title = "Bearing Noise Probability Distribution",
      .x_label = "Error [rad]",
      .y_label = "Probability Density [1/rad]",
  };

  std::vector<Data::Robot> &robots = data_.getRobots();

  for (; id < end_point; ++id) {
    std::string title = "Measurement PDF";

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string output_file = data_extraction_directory_ + title;
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    const unsigned short rows{2U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);

    gnuplot_ << gnuplot::grid();

    PlotList range_pdf;

    range_pdf.emplace_back(binary_robot_data_[id].range_pdf.filename,
                           binary_robot_data_[id].range_pdf.binary_format);
    range_pdf.back().settings = {
        .key_label = "Scaled PMF",
        .x = BIN_INDEX,
        .y = BIN_COUNT,
        .box_width = BIN_WIDTH,
        .style = gnuplot::BOXES,
    };

    double sigma = std::sqrt(robots[id].range_error.variance);
    double mu = robots[id].range_error.mean;

    std::ostringstream range_gaussian_plot;
    range_gaussian_plot << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
                        << "* exp(- " << "( x - " << mu << ")**2 / (2 *"
                        << std::pow(sigma, 2) << "))";

    range_pdf.emplace_back(range_gaussian_plot.str());
    range_pdf.back().settings = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    /* Add bearing PDF */
    PlotList bearing_pdf;

    bearing_pdf.emplace_back(binary_robot_data_[id].bearing_pdf.filename,
                             binary_robot_data_[id].bearing_pdf.binary_format);
    bearing_pdf.back().settings = {
        .key_label = "Scaled PMF",
        .x = BIN_INDEX,
        .y = BIN_COUNT,
        .box_width = BIN_WIDTH,
        .style = gnuplot::BOXES,
    };

    /* Plot Gaussian distribution over top of PDF. */
    sigma = std::sqrt(robots[id].bearing_error.variance);
    mu = robots[id].bearing_error.mean;

    std::ostringstream bearing_gaussian_plot;
    bearing_gaussian_plot << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
                          << "* exp(- " << "( x - " << mu << ")**2 / (2 *"
                          << std::pow(sigma, 2) << "))";

    bearing_pdf.emplace_back(bearing_gaussian_plot.str());
    bearing_pdf.back().settings = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    plot(range_pdf, range_axis);
    plot(bearing_pdf, bearing_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
}

void Plotter::addInferenceIteration() {

  std::initializer_list<PlotType> plots{SYNCED, ERROR, ABSOLUTE_ERROR};

  std::vector<Robot> &robots = data_.getRobots();

  for (const auto &plot : plots) {
    for (const Robot &robot : robots) {

      /* NOTE: The robot ID's are 1 indexed. */
      unsigned short id{static_cast<unsigned short>(robot.id - 1U)};

      std::string *inference_file{};
      const std::vector<Robot::State> *pose_data;

      switch (plot) {
      case SYNCED:
        inference_file = &binary_robot_data_[id].iterations.pose;
        *inference_file =
            "Robot" + std::to_string(robot.id) + "_synced_iterations";
        pose_data = &robot.synced.states;
        break;

      case ERROR:
        inference_file = &binary_robot_data_[id].iterations.error;
        *inference_file =
            "Robot" + std::to_string(robot.id) + "_error_iterations";
        pose_data = &robot.error.states;
        break;

      case ABSOLUTE_ERROR:
        inference_file = &binary_robot_data_[id].iterations.absolute_error;
        *inference_file =
            "Robot" + std::to_string(robot.id) + "_absolute_error_iterations";
        pose_data = &robot.absolute_state_error;
        break;

      default:
        throw std::runtime_error(
            "Plot type not supported for inference iteration");
      }

      /* Delete inference file from pervious run. */
      std::string file_location{"/tmp/" + *inference_file + ".bin"};
      if (total_inference_iterations_ == 0 &&
          std::filesystem::exists(file_location)) {
        std::filesystem::remove(file_location);
      }

      write_binary(*inference_file, *pose_data, true);
    }
  }

  ++total_inference_iterations_;
}

void Plotter::plotInferenceIterations(std::initializer_list<PlotType> plots,
                                      std::vector<unsigned int> iterations) {
  terminal_.number = ++terminal_number_;
  gnuplot_ << gnuplot::setTerminal(terminal_);
  gnuplot_ << gnuplot::grid();

  const unsigned short rows{3U}, columns{1U};

  for (const auto &plot : plots) {
    gnuplot_ << gnuplot::setMultiplot(rows, columns);
    inferenceIterationsPlotter(plot, iterations);
    gnuplot_ << gnuplot::unsetMultiplot();
  }
  gnuplot_.flush();
}

void Plotter::inferenceIterationsPlotter(PlotType plot_type,
                                         std::vector<unsigned int> iterations) {
  assert(total_inference_iterations_ > 0);
  binariseRobotPoseData({GROUNDTRUTH}, 1);

  std::vector<bool> plot_iteration(total_inference_iterations_, false);

  /* If the user does not provide the iterations they want to plot, then all the
   * iterations will be plotted. */
  if (iterations.empty()) {
    std::fill(plot_iteration.begin(), plot_iteration.end(), true);
  } else {

    for (const auto &iteration : iterations) {
      assert((iteration < total_inference_iterations_) && (iteration >= 0));
      plot_iteration[iteration] = true;
    }
  }

  gnuplot::AxisSettings x_axis{.x_label = "Time [s]",
                               .y_label = "x position [m]"},
      y_axis{.x_label = "Time [s]", .y_label = "y position [m]"},
      heading_axis{.x_label = "Time [s]", .y_label = "heading [rad]"};

  std::string *inference_file;

  switch (plot_type) {
  case SYNCED:
    inference_file = &binary_robot_data_[0U].iterations.pose;
    x_axis.title = "x position";
    y_axis.title = "y position";
    heading_axis.title = "heading";
    break;

  case ERROR:
    inference_file = &binary_robot_data_[0U].iterations.error;
    x_axis.title = "x position error";
    y_axis.title = "y position error";
    heading_axis.title = "heading error";

    break;
  case ABSOLUTE_ERROR:
    inference_file = &binary_robot_data_[0U].iterations.absolute_error;

    x_axis.title = "Absolute x position error";
    y_axis.title = "Absolute y position error";
    heading_axis.title = "Absolute heading error";
    break;

  default:
    throw std::runtime_error("Plot type provided to the inference iterations "
                             "plotter is not accepted");
  }

  std::vector<Robot> &robots = data_.getRobots();

  PlotList x_plots, y_plots, heading_plots;

  if (plot_type == SYNCED) {
    std::string groundtruth_file = binary_robot_data_[0U].pose.groundtruth;

    x_plots.emplace_back(groundtruth_file, RobotData::Pose::binary_format());
    x_plots.back().settings = {
        .key_label = "Groundtruth",
        .x = TIME,
        .y = X_POSITION,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::LIGHT_RED,
    };

    y_plots.emplace_back(groundtruth_file, RobotData::Pose::binary_format());
    y_plots.back().settings = {
        .key_label = "Groundtruth",
        .x = TIME,
        .y = Y_POSITION,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::LIGHT_RED,
    };

    heading_plots.emplace_back(groundtruth_file,
                               RobotData::Pose::binary_format());
    heading_plots.back().settings = {
        .key_label = "Groundtruth",
        .x = TIME,
        .y = HEADING,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::LIGHT_RED,
    };
  }

  for (unsigned int i{}; i < total_inference_iterations_; ++i) {

    /* Check if the current iteration of the optimiser should be plotted. */
    if (!plot_iteration[i])
      continue;

    /* HACK: It seems like the binary data saves the double values as
     * floating point values represented by 32 bits. */
    const unsigned short bits_per_item{32U};

    x_plots.emplace_back(*inference_file, RobotData::Pose::binary_format());
    x_plots.back().settings = {
        .key_label = std::to_string(i),
        .x = TIME,
        .y = X_POSITION,
        .style = gnuplot::LINES,
        .linecolor = (i == 0U) ? gnuplot::BLACK : gnuplot::NONE,
        .record = data_points_,
        .skip = i * data_points_ * bits_per_item,
    };

    y_plots.emplace_back(*inference_file, RobotData::Pose::binary_format());
    y_plots.back().settings = {
        .key_label = std::to_string(i),
        .x = TIME,
        .y = Y_POSITION,
        .style = gnuplot::LINES,
        .linecolor = (i == 0U) ? gnuplot::BLACK : gnuplot::NONE,
        .record = data_points_,
        .skip = i * data_points_ * bits_per_item,
    };

    heading_plots.emplace_back(*inference_file,
                               RobotData::Pose::binary_format());
    heading_plots.back().settings = {
        .key_label = std::to_string(i),
        .x = TIME,
        .y = HEADING,
        .style = gnuplot::LINES,
        .linecolor = (i == 0U) ? gnuplot::BLACK : gnuplot::NONE,
        .record = data_points_,
        .skip = i * data_points_ * bits_per_item,
    };
  }

  plot(x_plots, x_axis);
  plot(y_plots, y_axis);
  plot(heading_plots, heading_axis);
}

void Plotter::trajectoryIterationsPlotter(
    unsigned short robot_id, std::vector<unsigned int> iterations) {

  assert(total_inference_iterations_ > 0U);

  std::vector<bool> plot_iteration(total_inference_iterations_, false);

  /* If the user does not provide the iterations they want to plot, then all the
   * iterations will be plotted. */
  if (iterations.empty()) {
    std::fill(plot_iteration.begin(), plot_iteration.end(), true);
  } else {

    for (const auto &iteration : iterations) {
      assert((iteration < total_inference_iterations_) && (iteration >= 0));
      plot_iteration[iteration] = true;
    }
  }

  unsigned short end_point{(robot_id == 0) ? total_robots_ : robot_id};

  unsigned short id{
      static_cast<unsigned short>((robot_id == 0) ? 0 : robot_id - 1)};

  for (; id < end_point; ++id) {
    std::string title{"Robot " + std::to_string(id + 1) + " Trajectory"};
    std::string output_file{data_extraction_directory_ + title};

    gnuplot::AxisSettings axis{
        .title = title,
        .x_label = "x position [m]",
        .y_label = "y position [m]",
    };

    PlotList plot_list;

    for (unsigned short i{}; i < total_inference_iterations_; ++i) {

      plot_list.emplace_back(binary_landmark_data_.filename,
                             RobotData::Iterations::binary_format());

      plot_list.back().settings = {
          .key_label = "Landmarks",
          .style = gnuplot::PlotStyle::POINTS,
      };

      plot_list.emplace_back(binary_robot_data_[id].pose.groundtruth,
                             RobotData::Iterations::binary_format());
      plot_list.back().settings = {
          .key_label = "Groundtruth Trajectory",
          .x = X_POSITION,
          .y = Y_POSITION,
          .style = gnuplot::PlotStyle::LINES,
      };

      plot_list.emplace_back(binary_robot_data_[id].iterations.pose,
                             RobotData::Iterations::binary_format());

      /* HACK: It seems like the binary data saves the double values as
       * floating point values represented by 32 bits. */
      const unsigned short bits_per_item{32U};

      plot_list.back().settings = {
          .key_label = "Inferred Trajectory",
          .x = X_POSITION,
          .y = Y_POSITION,
          .style = gnuplot::PlotStyle::LINES,
          .linecolor = (i == 0U) ? gnuplot::BLACK : gnuplot::NONE,
          .record = data_points_,
          .skip = i * data_points_ * bits_per_item,
      };
    }
    plot(plot_list, axis);

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
 * Writes binary file containing robot pose (state) data such that gnuplot can
 * use it for plotting.
 * @param filename the name of the output binary file.
 * @param state_data Vector of measurements.
 */
void Plotter::write_binary(std::string &filename,
                           const std::vector<Robot::State> &state_data,
                           bool append) {

  /* Prepend temporary directory to filename. */
  filename = "/tmp/" + filename + ".bin";

#ifdef REUSE
  if (std::filesystem::exists(filename)) {
    return;
  }
#endif // REUSE
  std::_Ios_Openmode open_mode =
      append ? (std::ios::binary | std::ios::app) : std::ios::binary;

  /* Convert odometry data to binary. */
  std::ofstream fout(filename, open_mode);
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
 * Writes binary file containing robot measurement data such that gnuplot can
 * use it for plotting.
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
 * Writes binary file containing landmark position data such that gnuplot can
 * use it for plotting.
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
 * @param plots list of plots that gnuplot should plot on a SINGLE axis.
 * @param axis_settings axis settings for the plot.
 * @note this function plots all plots on a single axis. For multiplots, the
 * function needs to be wrapped in a multiplot and called multiple times.
 */
void Plotter::plot(const PlotList &plots,
                   const gnuplot::AxisSettings &axis_settings) {

  size_t total_plots = plots.size();

  gnuplot_ << gnuplot::setAxisSettings(axis_settings);
  gnuplot_ << gnuplot::setTitle(axis_settings.title);

  std::ostringstream plot_command;

  plot_command << "plot ";

  for (size_t i{}; i < total_plots; ++i) {

    if (plots[i].using_binary_file) {
      assert((plots[i].binary_name != "") && "Binary file not set.");

      plot_command << "'" << plots[i].binary_name << "'" << " binary "
                   << "format='" << plots[i].binary_format << "' ";

    } else {
      plot_command << plots[i].plot_string << " ";
    }

    plot_command << gnuplot::setPlotSettings(plots[i].settings);

    if (i < total_plots - 1) {
      plot_command << ",";
    }
  }

  plot_command << "\n";

  gnuplot_ << plot_command.str();

#ifdef DEBUG
  std::cout << plot_command.str() << std::endl;
#endif // DEBUG
}

} // namespace Data
