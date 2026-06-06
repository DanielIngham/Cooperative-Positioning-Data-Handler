#include "UtiasMrclam/Plotter.hpp"
#include "UtiasMrclam/DataHandler.hpp"
#include "UtiasMrclam/agents/Landmark.hpp"
#include "UtiasMrclam/agents/Robot.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#ifdef REUSE
#include <filesystem>
#endif // REUSE

namespace utias::mrclam {

unsigned short Plotter::terminal_number_{};

/**
 * Constructor for instance of the Plotter which sets its fields using the data
 * extracted by the Handler.
 * @param data Instance of the Handler class that has been assigned a dataset.
 * @note NOTE: At this state, the plotter requires the data Handler to have its
 * dataset set before creating an instance of the plotter.
 */
Plotter::Plotter() {

  output_directory_ = Handler::getProjectDirectory() + Handler::output_folder;
  plots_directory_ = output_directory_ + "/plots/";
}

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
template <typename T>
PlotData<T> Plotter::binariseData(const Agent &agent,
                                  const std::vector<T> &data) {

  PlotData<T> plot{agent};

  plot.data_.binary_filename_ = generate_temp_filename();

  write_binary(plot.data_.binary_filename_, data);

  return plot;
}

PlotData<Robot::Measurement> Plotter::binariseMeasurementVectors(
    const Agent &agent, const std::vector<Robot::Measurement> &measurements,
    const std::vector<Robot::State> &poses) {

  PlotData<Robot::Measurement> vector_plot{agent};

  vector_plot.data_ = {
      .binary_filename_ = generate_temp_filename(),
      .binary_file_format_ = "%double%double%double%double",
  };

  const std::string &filename{vector_plot.data_.binary_filename_};
  std::ofstream fout(filename, std::ios::binary);
  if (!fout) {
    throw std::runtime_error("Could not open temporary file");
  }

  /* Loop through each measurement. */
  for (const Robot::Measurement &measurement : measurements) {

    const size_t total_datapoints{poses.size()};

    const Robot::State *current_pose;

    /* Find the index in the ground truth that corresponds to the measurement
     * time. */
    for (size_t k{}; k < total_datapoints; ++k) {
      current_pose = &poses.at(k);

      /* Round to 4 decimal places */
      static constexpr double decimal_threshold{1e-4};

      if (std::abs(current_pose->time - measurement.time) < decimal_threshold) {
        break;
      }
    }

    for (unsigned int i{}; i < measurement.subjects.size(); i++) {

      const double y{
          measurement.ranges[i] *
          std::sin(measurement.bearings[i] + current_pose->orientation)};

      const double x{
          measurement.ranges[i] *
          std::cos(measurement.bearings[i] + current_pose->orientation)};

      /* Convert odometry data to binary. */
      fout.write(reinterpret_cast<const char *>(&current_pose->x),
                 sizeof(double));
      fout.write(reinterpret_cast<const char *>(&current_pose->y),
                 sizeof(double));
      fout.write(reinterpret_cast<const char *>(&x), sizeof(double));
      fout.write(reinterpret_cast<const char *>(&y), sizeof(double));
    }
  }

  fout.close();

  return vector_plot;
}
/**
 * Groups the robots measurement error for its range and bearing sensing, and
 * creates a quasi PDF (scaled PMF).
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @param bin_size Size of the bins in which the error values are grouped.
 * @note If the ID number is 0, all robot plots will be shown.
 */
std::pair<PlotData<Robot::Measurement>, PlotData<Robot::Measurement>>
Plotter::binariseMeasurementPDF(
    const Agent &agent, const std::vector<Robot::Measurement> &measurements,
    const double bin_size) {

  std::pair<PlotData<Robot::Measurement>, PlotData<Robot::Measurement>>
      range_bearing_plots{agent, agent};

  range_bearing_plots.first.data_.binary_file_format_ = "%double%double%double",
  range_bearing_plots.second.data_.binary_file_format_ =
      "%double%double%double";

  std::unordered_map<int, double> range_bin_counts;
  std::unordered_map<int, double> bearing_bin_counts;

  /* Create PDFs. */
  const double number_of_measurements{
      static_cast<double>(Handler::getNumberOfMeasurements(measurements))};

  for (const auto &data : measurements) {

    for (unsigned short i{}; i < data.subjects.size(); ++i) {

      int range_bin_index{
          static_cast<int>(std::floor(data.ranges[i] / bin_size))};

      int bearing_bin_index{
          static_cast<int>(std::floor(data.bearings[i] / bin_size))};

      range_bin_counts[range_bin_index] +=
          1.0 / (number_of_measurements * bin_size);

      bearing_bin_counts[bearing_bin_index] +=
          1.0 / (number_of_measurements * bin_size);
    }
  }

  std::string &range_filename{range_bearing_plots.first.data_.binary_filename_};
  range_filename = generate_temp_filename();
  write_binary(range_filename, range_bin_counts, bin_size);

  std::string &bearing_filename{
      range_bearing_plots.second.data_.binary_filename_};
  bearing_filename = generate_temp_filename();
  write_binary(bearing_filename, bearing_bin_counts, bin_size);

  return range_bearing_plots;
}

/**
 * Groups the robots odometry error for its range and bearing sensing, and
 * creates a quasi PDF (scaled PMF).
 * @param robot_id ID number of the robot whose plot the user wants to see.
 * @param bin_size Size of the bins in which the error values are grouped.
 * @note If the ID number is 0, all robot plots will be shown.
 */
std::pair<PlotData<Robot::Odometry>, PlotData<Robot::Odometry>>
Plotter::binariseOdometryPDF(const Agent &agent,
                             const std::vector<Robot::Odometry> &odometries,
                             const double bin_size) {

  std::pair<PlotData<Robot::Odometry>, PlotData<Robot::Odometry>>
      forward_angular_plots{agent, agent};

  forward_angular_plots.first.data_.binary_file_format_ =
      "%double%double%double",
  forward_angular_plots.second.data_.binary_file_format_ =
      "%double%double%double";

  std::unordered_map<int, double> forward_velocity_bin_counts;
  std::unordered_map<int, double> angular_velocity_bin_counts;

  /* Create PDFs for forward and angular velocity. */
  for (const auto &odometry : odometries) {
    int forward_velocity_bin_index{
        static_cast<int>(std::floor(odometry.forward_velocity / bin_size))};

    int angular_velocity_bin_index{
        static_cast<int>(std::floor(odometry.angular_velocity / bin_size))};

    /* NOTE: The bin count is actually the area contribution of the odometry
     * error for the given measurement. This means that the output is a
     * discretized pdf, where the sum of the area of all the bins should
     * equal 1. This is done for better visualisation when fitting a
     * Gaussian curve to the data. */
    forward_velocity_bin_counts[forward_velocity_bin_index] +=
        1.0 / (odometries.size() * bin_size);

    angular_velocity_bin_counts[angular_velocity_bin_index] +=
        1.0 / (odometries.size() * bin_size);
  }

  std::string &forward_velocity_filename{
      forward_angular_plots.first.data_.binary_filename_};

  forward_velocity_filename = generate_temp_filename();

  write_binary(forward_velocity_filename, forward_velocity_bin_counts,
               bin_size);

  std::string &angular_velocity_filename{
      forward_angular_plots.second.data_.binary_filename_};

  angular_velocity_filename = generate_temp_filename();

  write_binary(angular_velocity_filename, angular_velocity_bin_counts,
               bin_size);

  return forward_angular_plots;
}

/**
 * Creates a gif of the optimisation parameter values after each iteration.
 * @param plots the types of plots the should be created from the parameter
 * values.
 */
void Plotter::inferenceAnimation(const Robot &robot) {

  const auto iteration_types{getIterationsData(robot)};

  terminal_ = {
      .type = gnuplot::GIF,
  };

  for (const auto &[iterations, type] : iteration_types) {
    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    gnuplot_ << gnuplot::grid();

    std::string file_name{"Robot_" + robot.id() + "_" + to_string(type) +
                          "_iterations"};

    const std::string output_file{plots_directory_ + file_name};
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    const PlotData<Robot::State> &first_iteration{iterations.front()};

    for (const auto &iteration : iterations) {
      static constexpr unsigned short rows{3U}, columns{1U};
      gnuplot_ << gnuplot::setMultiplot(rows, columns);

      inferenceIterationsPlotter(first_iteration.agent_,
                                 {first_iteration, iteration}, type);

      gnuplot_ << gnuplot::unsetMultiplot();
    }

    gnuplot_.flush();
  }
}

void Plotter::trajectoryAnimation(const Robot &robot,
                                  const std::vector<Landmark> &landmarks) {

  const auto iteration_types{getIterationsData(robot)};

  const PlotData landmark_plot{binariseData(landmarks.front(), landmarks)};

  terminal_ = {
      .type = gnuplot::GIF,
  };

  for (const auto &[iterations, type] : iteration_types) {
    gnuplot_ << gnuplot::setTerminal(terminal_);
    gnuplot_ << gnuplot::grid();

    const std::string filename{"Robot_" + robot.id() + "trajectory_animation"};
    const std::string output_dirctory{plots_directory_ + filename};
    gnuplot_ << gnuplot::setOutput(output_dirctory, terminal_);

    const PlotData<Robot::State> &first_iteration{iterations.front()};

    for (const auto &iteration : iterations) {

      trajectoryIterationsPlotter(first_iteration.agent_,
                                  {first_iteration, iteration}, landmark_plot,
                                  type);
    }
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
void Plotter::plotTrajectory(const std::vector<Robot> &robots,
                             const std::vector<Landmark> &landmarks,
                             PlotTypeList types) {

  PlotData landmark_plot_data{binariseData(landmarks.front(), landmarks)};

  for (const auto &robot : robots) {

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    std::string filename{"Robot_" + robot.id() + "_Trajectory"};
    std::string output_file{plots_directory_ + filename};

    gnuplot_ << gnuplot::setOutput(output_file, terminal_);
    gnuplot_ << gnuplot::grid();

    gnuplot::AxisSettings axis{
        .title = "Robot " + robot.id() + " Trajectory",
        .x_label = "x position [m]",
        .y_label = "y position [m]",
    };

    PlotSettingsList plot_list;

    plot_list.emplace_back(landmark_plot_data.data_);
    plot_list.back().settings_ = {
        .key_label = "Landmarks",
        .style = gnuplot::PlotStyle::POINTS,
    };

    for (const auto &plot_type : types) {

      /* Create the plot for the trajectory of the robots. */
      std::string plot_file{};
      std::string key{};

      std::unique_ptr<PlotData<Robot::State>> trajectory_plot;

      switch (plot_type) {
      case Type::SYNCED:
        trajectory_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.synced.states));
        key = "Synced trajectory";
        break;

      case Type::GROUNDTRUTH:
        trajectory_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.groundtruth.states));
        key = "Groundtruth trajectory";
        break;

      default:
        throw std::runtime_error("Plot type not accepted for trajectory plot.");
      }

      plot_list.emplace_back(trajectory_plot->data_);
      plot_list.back().settings_ = {
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

void Plotter::plotMeasurementsVector(const std::vector<Robot> &robots,
                                     const std::vector<Landmark> &landmarks,
                                     PlotTypeList types) {

  PlotData<Landmark> landmark_plot{binariseData(landmarks.front(), landmarks)};

  for (const auto &robot : robots) {
    PlotData<Robot::State> gt_pose{
        binariseData(robot, robot.groundtruth.states)};

    PlotData<Robot::Measurement> vectors{binariseMeasurementVectors(
        robot, robot.groundtruth.measurements, robot.groundtruth.states)};

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    const std::string filename{"Robot_" + robot.id() + "_Measurement_Vectors"};
    const std::string output_file{plots_directory_ + filename};
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);
    gnuplot_ << gnuplot::grid();

    PlotSettingsList plots;

    plots.emplace_back(vectors.data_);
    plots.back().settings_ = {
        .key_label = "Measurement Vectors",
        .style = gnuplot::PlotStyle::VECTORS,
        .linecolor = gnuplot::Colour::LIGHT_RED,
    };

    plots.emplace_back(landmark_plot.data_);
    plots.back().settings_ = {
        .key_label = "Landmarks",
        .style = gnuplot::PlotStyle::POINTS,
        .pointtype = gnuplot::PointType::FILLED_PENTAGON,
        .pointsize = 4,
        .linecolor = gnuplot::Colour::BLACK,
    };

    plots.emplace_back(gt_pose.data_);

    plots.back().settings_ = {
        .x = X_POSITION,
        .y = Y_POSITION,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::Colour::BLACK,
        .linewidth = 2,
    };

    /* NOTE: the first two coordinates of the measurement vector correspond to
     * the instances of the groundtruth trajectory where a measurement was
     * taken.*/
    plots.emplace_back(vectors.data_);
    plots.back().settings_ = {
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
void Plotter::plotPoses(const std::vector<Robot> &robots, PlotTypeList types) {

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

  for (const Type &plot_type : types) {

    PlotSettingsList x_plots, y_plots, orientation_plots;

    bool empty_list{};
    for (const auto &robot : robots) {

      std::unique_ptr<PlotData<Robot::State>> pose_plot;
      std::string plot_title{"Robot " + robot.id()};

      switch (plot_type) {
      case Type::GROUNDTRUTH:
        pose_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.groundtruth.states));
        {
          x_position_axis.title = "Groundtruth ";
          y_position_axis.title = "Groundtruth ";
          orientation_axis.title = "Groundtruth ";
        }
        break;

      case Type::SYNCED:
        pose_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.synced.states));
        {
          x_position_axis.title = "Synced ";
          y_position_axis.title = "Synced ";
          orientation_axis.title = "Synced ";
        }
        break;

      case Type::RAW:
        /* NOTE: Simulated data does not have raw values. */
        if (robot.raw.states.empty()) {
          std::cerr << "\033[1;33m" << "[WARNING]" << "\033[0m "
                    << "Raw data not populated. Skipping plot." << std::endl;
          empty_list = true;

          continue;
        }

        pose_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.raw.states));
        {
          x_position_axis.title = "Raw ";
          y_position_axis.title = "Raw ";
          orientation_axis.title = "Raw ";
        }
        break;

      case Type::ERROR:
        pose_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.error.states));
        {
          x_position_axis.title = "Error ";
          y_position_axis.title = "Error ";
          orientation_axis.title = "Error ";
        }
        break;

      case Type::ABSOLUTE_ERROR:
        pose_plot = std::make_unique<PlotData<Robot::State>>(
            binariseData(robot, robot.absolute_state_error));
        {
          x_position_axis.title = "Absolute Error ";
          y_position_axis.title = "Absolute Error ";
          orientation_axis.title = "Absolute Error ";
        }
        break;

      default:
        break;
      }

      x_position_axis.title += "x Coordinate";
      y_position_axis.title += "x Coordinate";
      orientation_axis.title += "Orientation";

      static constexpr double point_size{0.5};

      x_plots.emplace_back(pose_plot->data_);
      x_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = X_POSITION,
          .style = gnuplot::LINESPOINTS,
          .pointsize = point_size,
      };

      y_plots.emplace_back(pose_plot->data_);
      y_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = Y_POSITION,
          .style = gnuplot::LINESPOINTS,
          .pointsize = point_size,
      };

      orientation_plots.emplace_back(pose_plot->data_);
      orientation_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = HEADING,
          .style = gnuplot::LINESPOINTS,
          .pointsize = point_size,
      };
    }

    if (empty_list)
      continue;

    gnuplot_ << gnuplot::setTerminal(terminal_);
    terminal_.number = ++terminal_number_;

    const std::string file_name{"Robot_" + to_string(plot_type) + "Poses"};
    const std::string output_file{plots_directory_ + file_name};
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    gnuplot_ << gnuplot::grid();

    static constexpr unsigned short rows{3U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);

    plot(x_plots, x_position_axis);
    plot(y_plots, y_position_axis);
    plot(orientation_plots, orientation_axis);

    gnuplot_ << gnuplot::unsetMultiplot();
    gnuplot_.flush();
  }
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
void Plotter::plotOdometry(const std::vector<Robot> &robots,
                           PlotTypeList types) {

  /* Set forward velocity axis labels */
  gnuplot::AxisSettings forward_velocity_axis{
      .x_label = "Time [s]",
      .y_label = "Forward Velocity [m/s]",
  };

  gnuplot::AxisSettings angular_velocity_axis{
      .x_label = "Time [s]",
      .y_label = "Angular Velocity [rad/s]",
  };

  for (const auto &robot : robots) {

    PlotSettingsList forward_velocity_plots, angular_velocity_plots;

    for (const Type &plot_type : types) {
      std::unique_ptr<PlotData<Robot::Odometry>> odometry_plot;
      std::string plot_title;

      switch (plot_type) {
      case Type::GROUNDTRUTH:
        odometry_plot = std::make_unique<PlotData<Robot::Odometry>>(
            binariseData(robot, robot.groundtruth.odometry));
        plot_title = "Groundtruth";
        break;

      case Type::SYNCED:
        odometry_plot = std::make_unique<PlotData<Robot::Odometry>>(
            binariseData(robot, robot.synced.odometry));
        plot_title = "Synced";
        break;

      case Type::RAW:
        odometry_plot = std::make_unique<PlotData<Robot::Odometry>>(
            binariseData(robot, robot.raw.odometry));
        plot_title = "Raw";
        break;

      case Type::ERROR:
        odometry_plot = std::make_unique<PlotData<Robot::Odometry>>(
            binariseData(robot, robot.error.odometry));
        plot_title = "Error";
        break;

      default:
        break;
      }

      /* Range plot */
      forward_velocity_plots.emplace_back(odometry_plot->data_);

      forward_velocity_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = FORWARD_VELOCITY,
      };

      angular_velocity_plots.emplace_back(odometry_plot->data_);

      angular_velocity_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = ANGULAR_VELOCITY,
      };
    }

    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);

    const std::string filename{"Robot " + robot.id() + " Odometry"};
    const std::string output_file{plots_directory_ + filename};
    gnuplot_ << gnuplot::setOutput(output_file, terminal_);

    static constexpr unsigned short rows{2U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);

    gnuplot_ << gnuplot::grid();

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
void Plotter::plotOdometryPDFs(const std::vector<Robot> &robots,
                               const double bin_size) {

  const gnuplot::AxisSettings forward_velocity_axis{
      .title = "Forward Velocity Probability Distribution",
      .x_label = "Error [m/s]",
      .y_label = "Probability Density [s/m]",
  };

  const gnuplot::AxisSettings angular_velocity_axis{
      .title = "Angular Velocity Noise Probability Distribution",
      .x_label = "Error [rad/s]",
      .y_label = "Probability Density [s/rad]",
  };

  for (const auto &robot : robots) {

    const auto &[forward_velocity, angular_velocity]{
        binariseOdometryPDF(robot, robot.error.odometry, bin_size)};

    PlotSettingsList forward_velocity_pdf;

    forward_velocity_pdf.emplace_back(forward_velocity.data_);

    forward_velocity_pdf.back().settings_ = {
        .key_label = "Scaled PMF",
        .x = 1,
        .y = 3,
        .box_width = 2,
        .style = gnuplot::BOXES,
    };

    double sigma{std::sqrt(robot.forward_velocity_error.variance)};
    double mu{robot.forward_velocity_error.mean};

    std::ostringstream forward_velocity_gaussian_plot;
    forward_velocity_gaussian_plot
        << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
        << "* exp(- " << "( x - " << mu << ")**2 / (2 *" << std::pow(sigma, 2)
        << "))";

    forward_velocity_pdf.emplace_back(forward_velocity_gaussian_plot.str());
    forward_velocity_pdf.back().settings_ = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    PlotSettingsList angular_velocity_pdf;

    angular_velocity_pdf.emplace_back(angular_velocity.data_);

    angular_velocity_pdf.back().settings_ = {
        .key_label = "Scaled PMF",
        .x = 1,
        .y = 3,
        .box_width = 2,
        .style = gnuplot::BOXES,
    };

    sigma = std::sqrt(robot.angular_velocity_error.variance);
    mu = robot.angular_velocity_error.mean;

    std::ostringstream angular_velocity_gaussian_plot;
    angular_velocity_gaussian_plot
        << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
        << "* exp(- " << "( x - " << mu << ")**2 / (2 *" << std::pow(sigma, 2)
        << "))";

    angular_velocity_pdf.emplace_back(angular_velocity_gaussian_plot.str());
    angular_velocity_pdf.back().settings_ = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    /* Peform plotting */
    {
      terminal_.number = ++terminal_number_;
      gnuplot_ << gnuplot::setTerminal(terminal_);

      const std::string filename{"Odometry_PDF"};
      const std::string output_file{plots_directory_ + filename};
      gnuplot_ << gnuplot::setOutput(output_file, terminal_);

      static constexpr unsigned short rows{2U}, columns{1U};
      gnuplot_ << gnuplot::setMultiplot(rows, columns);
      gnuplot_ << gnuplot::grid();

      plot(forward_velocity_pdf, forward_velocity_axis);
      plot(angular_velocity_pdf, angular_velocity_axis);

      gnuplot_ << gnuplot::unsetMultiplot();
      gnuplot_.flush();
    }
  }
}

/**
 * Plots the range and bearing measurements that each vehicle recieved through a
 * run in the dataset.
 * @param plots List containing the type of data extracted/calculated from the
 * dataset.
 * @param robot_id The id of the robot whose states will be plotted.
 * @note If the parameter is left empty or set to 0, then all robots data will
 * be plotted in individual terminals.
 */
void Plotter::plotMeasurements(const std::vector<Robot> &robots,
                               PlotTypeList types) {

  /* Set forward velocity axis labels */
  const gnuplot::AxisSettings range_axis{
      .x_label = "Time [s]",
      .y_label = "Range [m]",
  };

  const gnuplot::AxisSettings bearing_axis{
      .x_label = "Time [s]",
      .y_label = "Bearing [rad]",
  };

  for (const auto &robot : robots) {

    PlotSettingsList range_plots, bearing_plots;

    for (const Type &plot_type : types) {
      std::string plot_title;
      std::unique_ptr<PlotData<Robot::Measurement>> measurement_plot;

      switch (plot_type) {
      case Type::GROUNDTRUTH:
        measurement_plot = std::make_unique<PlotData<Robot::Measurement>>(
            binariseData(robot, robot.groundtruth.measurements));
        plot_title = "Groundtruth";
        break;

      case Type::SYNCED:
        measurement_plot = std::make_unique<PlotData<Robot::Measurement>>(
            binariseData(robot, robot.synced.measurements));
        plot_title = "Synced";
        break;

      case Type::RAW:
        measurement_plot = std::make_unique<PlotData<Robot::Measurement>>(
            binariseData(robot, robot.raw.measurements));
        plot_title = "Raw";
        break;

      case Type::ERROR:
        measurement_plot = std::make_unique<PlotData<Robot::Measurement>>(
            binariseData(robot, robot.error.measurements));
        plot_title = "Error";
        break;
      default:
        break;
      }

      /* Range plot */
      range_plots.emplace_back(measurement_plot->data_);
      range_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = RANGE,
      };

      bearing_plots.emplace_back(measurement_plot->data_);
      bearing_plots.back().settings_ = {
          .key_label = plot_title,
          .x = TIME,
          .y = BEARING,
      };
    }

    /* Create Plot */
    {
      terminal_.number = ++terminal_number_;
      gnuplot_ << gnuplot::setTerminal(terminal_);

      std::string filename{"Robot_" + robot.id() + "_Measurements"};
      std::string output_file{plots_directory_ + filename};
      gnuplot_ << gnuplot::setOutput(output_file, terminal_);

      static constexpr unsigned short rows{2U}, columns{1U};
      gnuplot_ << gnuplot::setMultiplot(rows, columns);

      gnuplot_ << gnuplot::grid();

      plot(range_plots, range_axis);
      plot(bearing_plots, bearing_axis);

      gnuplot_ << gnuplot::unsetMultiplot();
      gnuplot_.flush();
    }
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
void Plotter::plotMeasurementPDFs(const std::vector<Robot> &robots,
                                  const double bin_size) {

  const gnuplot::AxisSettings range_axis{
      .title = "Range Noise Probability Distribution",
      .x_label = "Error [m]",
      .y_label = "Probability Density [1/m]",
  };

  const gnuplot::AxisSettings bearing_axis{
      .title = "Bearing Noise Probability Distribution",
      .x_label = "Error [rad]",
      .y_label = "Probability Density [1/rad]",
  };

  for (const auto &robot : robots) {

    const auto &[range_plot, bearing_plot]{
        binariseMeasurementPDF(robot, robot.error.measurements, bin_size)};

    PlotSettingsList range_pdf, bearing_pdf;

    range_pdf.emplace_back(range_plot.data_);
    range_pdf.back().settings_ = {
        .key_label = "Scaled PMF",
        .x = BIN_INDEX,
        .y = BIN_COUNT,
        .box_width = BIN_WIDTH,
        .style = gnuplot::BOXES,
    };

    double sigma{std::sqrt(robot.range_error.variance)},
        mu{robot.range_error.mean};

    std::ostringstream range_gaussian_plot;
    range_gaussian_plot << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
                        << "* exp(- " << "( x - " << mu << ")**2 / (2 *"
                        << std::pow(sigma, 2) << "))";

    range_pdf.emplace_back(range_gaussian_plot.str());
    range_pdf.back().settings_ = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    bearing_pdf.emplace_back(bearing_plot.data_);
    bearing_pdf.back().settings_ = {
        .key_label = "Scaled PMF",
        .x = BIN_INDEX,
        .y = BIN_COUNT,
        .box_width = BIN_WIDTH,
        .style = gnuplot::BOXES,
    };

    /* Plot Gaussian distribution over top of PDF. */
    sigma = std::sqrt(robot.bearing_error.variance);
    mu = robot.bearing_error.mean;

    std::ostringstream bearing_gaussian_plot;
    bearing_gaussian_plot << " 1.0 /" << "( " << sigma << " * sqrt(2 * pi)) "
                          << "* exp(- " << "( x - " << mu << ")**2 / (2 *"
                          << std::pow(sigma, 2) << "))";

    bearing_pdf.emplace_back(bearing_gaussian_plot.str());
    bearing_pdf.back().settings_ = {
        .key_label =
            latexEnabled(terminal_.type)
                ? "$\\mathcal{N}(" + std::to_string(mu) + "," +
                      std::to_string(sigma) + ")$"
                : "N(" + std::to_string(mu) + "," + std::to_string(sigma) + ")",
        .style = gnuplot::LINES,
        .linewidth = 4,
        .math_expression = true,
    };

    /* Create Plot */
    {
      terminal_.number = ++terminal_number_;
      gnuplot_ << gnuplot::setTerminal(terminal_);

      const std::string title{"Robot_" + robot.id() + "_Measurement_PDF"};
      const std::string output_file{plots_directory_ + title};
      gnuplot_ << gnuplot::setOutput(output_file, terminal_);

      static constexpr unsigned short rows{2U}, columns{1U};
      gnuplot_ << gnuplot::setMultiplot(rows, columns);

      gnuplot_ << gnuplot::grid();

      plot(range_pdf, range_axis);
      plot(bearing_pdf, bearing_axis);

      gnuplot_ << gnuplot::unsetMultiplot();
      gnuplot_.flush();
    }
  }
}

void Plotter::addInferenceIteration(const Robot &robot) {

  PlotData synced_plot{binariseData(robot, robot.synced.states)};
  PlotData groundtruth_plot{binariseData(robot, robot.groundtruth.states)};
  PlotData error_plot{binariseData(robot, robot.error.states)};
  PlotData abs_error_plot{binariseData(robot, robot.absolute_state_error)};

  /* Use the memory address of the robot instance as a key. */
  std::uintptr_t key{getKeyForRobot(robot)};

  const auto it{iteration_data_.find(key)};
  if (it == iteration_data_.end()) {
    iteration_data_.insert({key, IterationData{
                                     .synced = {synced_plot},
                                     .groundtruth = {groundtruth_plot},
                                     .error = {error_plot},
                                     .absolute_error = {abs_error_plot},
                                 }});
  } else {
    it->second.synced.push_back(synced_plot);
    it->second.synced.push_back(groundtruth_plot);
    it->second.synced.push_back(error_plot);
    it->second.synced.push_back(abs_error_plot);
  }
}

void Plotter::plotInferenceIterations(const Robot &robot) {

  const auto iteration_types{getIterationsData(robot)};

  for (const auto &[iterations, type] : iteration_types) {
    terminal_.number = ++terminal_number_;
    gnuplot_ << gnuplot::setTerminal(terminal_);
    gnuplot_ << gnuplot::grid();

    static constexpr unsigned short rows{3U}, columns{1U};
    gnuplot_ << gnuplot::setMultiplot(rows, columns);
    inferenceIterationsPlotter(iterations.front().agent_, iterations, type);
    gnuplot_ << gnuplot::unsetMultiplot();

    gnuplot_.flush();
  }
}

void Plotter::inferenceIterationsPlotter(
    const Agent &agent, const PlotDataList<Robot::State> &plots,
    Type plot_type) {

  const gnuplot::AxisSettings x_axis{.x_label = "Time [s]",
                                     .y_label = "x position [m]"};

  const gnuplot::AxisSettings y_axis{.x_label = "Time [s]",
                                     .y_label = "y position [m]"};

  const gnuplot::AxisSettings heading_axis{.x_label = "Time [s]",
                                           .y_label = "heading [rad]"};

  PlotSettingsList x_plots, y_plots, heading_plots;

  if (plot_type == Type::SYNCED) {
    const Robot *robot_ptr{dynamic_cast<const Robot *>(&agent)};

    PlotData<Robot::State> gt_data{
        binariseData(*robot_ptr, robot_ptr->groundtruth.states)};

    x_plots.emplace_back(gt_data.data_);
    x_plots.back().settings_ = {
        .key_label = "Groundtruth",
        .x = TIME,
        .y = X_POSITION,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::LIGHT_RED,
    };

    y_plots.emplace_back(gt_data.data_);
    y_plots.back().settings_ = {
        .key_label = "Groundtruth",
        .x = TIME,
        .y = Y_POSITION,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::LIGHT_RED,
    };

    heading_plots.emplace_back(gt_data.data_);
    heading_plots.back().settings_ = {
        .key_label = "Groundtruth",
        .x = TIME,
        .y = HEADING,
        .style = gnuplot::LINES,
        .linecolor = gnuplot::LIGHT_RED,
    };
  }

  unsigned short counter{};
  for (const auto &iteration : plots) {

    x_plots.emplace_back(iteration.data_);
    x_plots.back().settings_ = {
        .key_label = std::to_string(counter),
        .x = TIME,
        .y = X_POSITION,
        .style = gnuplot::LINES,
        .linecolor = (counter == 0U) ? gnuplot::BLACK : gnuplot::NONE,
    };

    y_plots.emplace_back(iteration.data_);
    y_plots.back().settings_ = {
        .key_label = std::to_string(counter),
        .x = TIME,
        .y = Y_POSITION,
        .style = gnuplot::LINES,
        .linecolor = (counter == 0U) ? gnuplot::BLACK : gnuplot::NONE,
    };

    heading_plots.emplace_back(iteration.data_);
    heading_plots.back().settings_ = {
        .key_label = std::to_string(counter),
        .x = TIME,
        .y = HEADING,
        .style = gnuplot::LINES,
        .linecolor = (counter == 0U) ? gnuplot::BLACK : gnuplot::NONE,
    };

    ++counter;
  }

  plot(x_plots, x_axis);
  plot(y_plots, y_axis);
  plot(heading_plots, heading_axis);
}

void Plotter::trajectoryIterationsPlotter(
    const Agent &agent, const PlotDataList<Robot::State> &iterations,
    const PlotData<Landmark> &landmark, Type type) {

  const gnuplot::AxisSettings axis{
      .title = "Robot " + agent.id() + " Trajectory",
      .x_label = "x position [m]",
      .y_label = "y position [m]",
  };

  PlotSettingsList plot_list;

  plot_list.emplace_back(landmark.data_);
  plot_list.back().settings_ = {
      .key_label = "Landmarks",
      .style = gnuplot::PlotStyle::POINTS,
  };

  const Robot *robot_ptr{dynamic_cast<const Robot *>(&agent)};
  const PlotData<Robot::State> gt_trajectory{
      binariseData(*robot_ptr, robot_ptr->groundtruth.states)};

  plot_list.emplace_back(gt_trajectory.data_);
  plot_list.back().settings_ = {
      .key_label = "Groundtruth Trajectory",
      .x = X_POSITION,
      .y = Y_POSITION,
      .style = gnuplot::PlotStyle::LINES,
  };

  unsigned short counter{};
  for (const auto &iteration : iterations) {

    plot_list.emplace_back(iteration.data_);

    plot_list.back().settings_ = {
        .key_label = std::to_string(counter),
        .x = X_POSITION,
        .y = Y_POSITION,
        .style = gnuplot::PlotStyle::LINES,
        .linecolor = (counter == 0U) ? gnuplot::BLACK : gnuplot::NONE,
    };
  }

  plot(plot_list, axis);
}

/**
 * Writes binary file for odometry data.
 * @param filename the name of the output binary file.
 * @param odometry_data Vector of measurements.
 */
void Plotter::write_binary(const std::string &filename,
                           const std::vector<Robot::Odometry> &odometry_data) {

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
void Plotter::write_binary(const std::string &filename,
                           const std::vector<Robot::State> &state_data,
                           bool append) {

#ifdef REUSE
  if (std::filesystem::exists(filename)) {
    return;
  }
#endif // REUSE
  std::_Ios_Openmode open_mode{append ? (std::ios::binary | std::ios::app)
                                      : std::ios::binary};

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
    const std::string &filename,
    const std::vector<Robot::Measurement> &measurement_data) {

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
    for (unsigned short i{}; i < row.subjects.size(); ++i) {
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
void Plotter::write_binary(const std::string &filename,
                           const std::vector<Landmark> &landmark_data) {

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

  for (const auto &row : landmark_data) {
    const double x{row.x()};
    const double y{row.y()};
    fout.write(reinterpret_cast<const char *>(&x), sizeof(double));
    fout.write(reinterpret_cast<const char *>(&y), sizeof(double));
  }

  fout.close();
}

void Plotter::write_binary(const std::string &filename,
                           const std::unordered_map<int, double> &pdf_data,
                           const double &bin_size) {

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
void Plotter::plot(const PlotSettingsList &plots,
                   const gnuplot::AxisSettings &axis_settings) {

  const size_t total_plots{plots.size()};

  gnuplot_ << gnuplot::setAxisSettings(axis_settings);
  gnuplot_ << gnuplot::setTitle(axis_settings.title);

  std::ostringstream plot_command;

  plot_command << "plot ";

  for (auto it = plots.begin(); it != plots.end(); ++it) {

    /* If a plot string has been provided, then that will be prioritized. */
    if (it->data_.plot_string_ == "") {
      assert((it->data_.binary_filename_ != "") && "Binary file not set.");

      plot_command << "'" << it->data_.binary_filename_ << "'"
                   << " binary "
                   << "format='" << it->data_.binary_file_format_ << "' ";

    } else {
      plot_command << it->data_.plot_string_ << " ";
    }

    plot_command << gnuplot::setPlotSettings(it->settings_);

    if (std::next(it) != plots.end()) {
      plot_command << ",";
    }
  }

  plot_command << "\n";

  gnuplot_ << plot_command.str();

#ifdef DEBUG
  std::cout << plot_command.str() << std::endl;
#endif // DEBUG
}

std::string Plotter::generate_temp_filename(const std::string &extension) {
  static std::random_device rd;
  static std::mt19937 gen{rd()};
  static std::uniform_int_distribution<> dis{0, 15};

  std::stringstream name;
  for (int i{}; i < 16; ++i) {
    name << std::hex << dis(gen);
  }

  auto temp_dir = std::filesystem::temp_directory_path();
  return (temp_dir / (name.str() + extension)).string();
}

std::uintptr_t Plotter::getKeyForRobot(const Robot &robot) {
  return reinterpret_cast<std::uintptr_t>(&robot);
}

std::array<std::pair<const Plotter::PlotDataList<Robot::State> &, Type>, 4U>
Plotter::getIterationsData(const Robot &robot) {

  std::uintptr_t key{getKeyForRobot(robot)};

  const auto it{iteration_data_.find(key)};

  if (it == iteration_data_.end())
    throw std::runtime_error(
        "Robot " + robot.id() +
        " has not inference iterations added to this plotter instance.");

  const IterationData &robot_iterations{it->second};

  std::array<std::pair<const PlotDataList<Robot::State> &, Type>, 4U>
      iteration_types{
          {{robot_iterations.synced, Type::SYNCED},
           {robot_iterations.groundtruth, Type::GROUNDTRUTH},
           {robot_iterations.error, Type::ERROR},
           {robot_iterations.absolute_error, Type::ABSOLUTE_ERROR}}};

  return iteration_types;
}

std::string Plotter::to_string(Type type) {
  switch (type) {
  case Type::SYNCED:
    return "synced";
  case Type::GROUNDTRUTH:
    return "groundtruth";
  case Type::RAW:
    return "raw";
  case Type::ERROR:
    return "error";
  case Type::ABSOLUTE_ERROR:
    return "absolute_error";
  default:
    throw std::invalid_argument("Unknown type provide Plotter::to_string.");
  }
}

} // namespace utias::mrclam
