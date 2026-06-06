/**
 * @file Plotter.h
 * @brief Contains gnuplot functionality.
 * @date 2025-08-06
 */
#pragma once

#include "UtiasMrclam/agents/Landmark.hpp"
#include "UtiasMrclam/agents/Robot.hpp"

#include <cmath>
#include <stdexcept>
#include <unistd.h>
#include <unordered_map>
#include <vector>

/* Warn about depreciated functions. */
#define GNUPLOT_DEPRECATE_WARN
#include "gnuplot/gnuplot-iostream.h"
#include "gnuplot/gnuplot_helper.h"

namespace utias::mrclam {

enum class Type { SYNCED, GROUNDTRUTH, RAW, ERROR, ABSOLUTE_ERROR };

class Plotter;

struct Data {

  std::string binary_filename_{};

  std::string binary_file_format_{};

  std::string plot_string_{};
};

template <typename T> class PlotData {
public:
  explicit PlotData(const Agent &agent) : agent_{agent} {
    data_.binary_file_format_ = setBinaryFormat();
  };

  PlotData(PlotData &&) = default;
  PlotData(const PlotData &) = default;
  PlotData &operator=(PlotData &&) = default;
  PlotData &operator=(const PlotData &) = default;
  ~PlotData() = default;

private:
  friend Plotter;

  const Agent &agent_;

  std::string name_;

  Data data_;

  std::string setBinaryFormat() {
    if constexpr (std::is_same_v<T, Robot::State>) {
      return "%double%double%double%double";

    } else if constexpr (std::is_same_v<T, Robot::Odometry>) {
      return "%double%double%double";

    } else if constexpr (std::is_same_v<T, Robot::Measurement>) {
      return "%double%ushort%double%double";

    } else if constexpr (std::is_same_v<T, Landmark>) {
      return "%double%double";

    } else {
      throw std::runtime_error("Unknown type provided to Plot template.");
    }
  }
};

struct PlotSettings {
  PlotSettings(std::string equation) { data_.plot_string_ = equation; };

  PlotSettings(Data data) : data_{data} {};

  Data data_;

  gnuplot::PlotSettings settings_;
};

class Plotter {
public:
  Plotter();
  Plotter(Plotter &&) = delete;
  Plotter(const Plotter &) = delete;
  Plotter &operator=(Plotter &&) = delete;
  Plotter &operator=(const Plotter &) = delete;
  ~Plotter() = default;

  using PlotSettingsList = std::vector<PlotSettings>;
  using PlotTypeList = std::initializer_list<Type>;
  template <typename T> using PlotDataList = std::vector<PlotData<T>>;

  void setTerminal(gnuplot::TerminalSettings);

  static constexpr PlotTypeList trajectory_types{Type::GROUNDTRUTH,
                                                 Type::SYNCED};

  static constexpr PlotTypeList sensor_types{Type::RAW, Type::GROUNDTRUTH,
                                             Type::SYNCED};

  static constexpr PlotTypeList pose_types{Type::GROUNDTRUTH, Type::SYNCED,
                                           Type::RAW, Type::ERROR,
                                           Type::ABSOLUTE_ERROR};

  void plotPoses(const std::vector<Robot> &, PlotTypeList types = pose_types);

  void plotTrajectory(const std::vector<Robot> &, const std::vector<Landmark> &,
                      PlotTypeList types = trajectory_types);

  void plotOdometry(const std::vector<Robot> &,
                    PlotTypeList types = sensor_types);

  void plotMeasurementsVector(const std::vector<Robot> &,
                              const std::vector<Landmark> &,
                              PlotTypeList types = trajectory_types);

  void plotMeasurements(const std::vector<Robot> &,
                        PlotTypeList types = sensor_types);

  void plotOdometryPDFs(const std::vector<Robot> &,
                        const double bin_size = 0.001);

  void plotMeasurementPDFs(const std::vector<Robot> &,
                           const double bin_size = 0.001);

  void inferenceAnimation(const Robot &);

  void addInferenceIteration(const Robot &);

  void trajectoryAnimation(const Robot &, const std::vector<Landmark> &);

  void plotInferenceIterations(const Robot &);

private:
  /** Data structure containing the settings of a gnuplot terminal. */
  gnuplot::TerminalSettings terminal_;

  /** The number of terminals that have been spawned by the
   * plotter. */
  static unsigned short terminal_number_;

  /** Directory where the data extraction plots are saved. */
  std::string output_directory_;
  std::string plots_directory_;

  /** Instance of the gnuplot iostream class that allows for plotting */
  Gnuplot gnuplot_;

  struct IterationData {
    PlotDataList<Robot::State> synced;
    PlotDataList<Robot::State> groundtruth;
    PlotDataList<Robot::State> error;
    PlotDataList<Robot::State> absolute_error;
  };

  std::unordered_map<std::uintptr_t, IterationData> iteration_data_;

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
    HEADING = 4,
    /* PDF */
    BIN_INDEX = 1,
    BIN_WIDTH = 2,
    BIN_COUNT = 3,
  };

  void plot(const PlotSettingsList &, const gnuplot::AxisSettings &);

  template <typename T>
  PlotData<T> binariseData(const Agent &, const std::vector<T> &);

  PlotData<Robot::Measurement>
  binariseMeasurementVectors(const Agent &,
                             const std::vector<Robot::Measurement> &,
                             const std::vector<Robot::State> &);

  std::pair<PlotData<Robot::Measurement>, PlotData<Robot::Measurement>>
  binariseMeasurementPDF(const Agent &, const std::vector<Robot::Measurement> &,
                         const double);

  std::pair<PlotData<Robot::Odometry>, PlotData<Robot::Odometry>>
  binariseOdometryPDF(const Agent &, const std::vector<Robot::Odometry> &,
                      const double);

  void write_binary(const std::string &, const std::vector<Robot::Odometry> &);
  void write_binary(const std::string &, const std::vector<Robot::State> &,
                    bool append = false);
  void write_binary(const std::string &,
                    const std::vector<Robot::Measurement> &);
  void write_binary(const std::string &, const std::vector<Landmark> &);
  void write_binary(const std::string &,
                    const std::unordered_map<int, double> &, const double &);

  void inferenceIterationsPlotter(const Agent &,
                                  const PlotDataList<Robot::State> &, Type);

  void trajectoryIterationsPlotter(const Agent &,
                                   const PlotDataList<Robot::State> &,
                                   const PlotData<Landmark> &, Type);

  std::string generate_temp_filename(const std::string &extension = ".bin");

  std::array<std::pair<const PlotDataList<Robot::State> &, Type>, 4U>
  getIterationsData(const Robot &);

  std::uintptr_t getKeyForRobot(const Robot &);

  std::string to_string(Type);
};

} // namespace utias::mrclam
