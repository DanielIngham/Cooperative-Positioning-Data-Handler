/**
 * @file Simulator.h
 * @brief Header file of the Simulator class,
 * @author Daniel Ingham
 * @date 2025-04-25
 */
#ifndef INCLUDE_INCLUDE_SIMULATOR_H_
#define INCLUDE_INCLUDE_SIMULATOR_H_

#include "Landmark.h"
#include "Robot.h"

#include <cmath>
#include <random>
#include <vector>

namespace Data {

struct SimulationDefaults {
  static constexpr unsigned short kRobots{5U};
  static constexpr unsigned short kLandmarks{15U};
  static constexpr double kSamplePeriod{0.02};
  static constexpr unsigned long kSeed{0U};
  static const inline std::string kOutputDir{};

  /* The measurement sensor is slower than the odometry sensor, so the is used
   * to determine when a measurment should be taken. */
  static constexpr unsigned short kmeasurement_to_odometry_ratio{5U};
  static constexpr double kmax_range{4.0};
};

/**
 * @class Simulator
 * @brief Populates Robot and Landmark vectors with simulated values.
 */
class Simulator {
public:
  Simulator();
  Simulator(const unsigned long int, double, std::vector<Robot> &,
            std::vector<Landmark> &,
            const unsigned long seed = SimulationDefaults::kSeed);
  ~Simulator();

  /* Setters */
  void setSimulation(const unsigned long int, double, std::vector<Robot> &,
                     std::vector<Landmark> &,
                     const unsigned long seed = SimulationDefaults::kSeed);

private:
  /**
   * Seed value used for the generator.
   */
  size_t generator_seed_;

  /**
   * @ brief The total number of samples for each robot in the simulation.
   */
  unsigned long int data_points_{};

  /**
   * @brief The period between samples for the simulated groundtruth and
   * odometry readings.
   */
  double sample_period_{0.02};

  double measurement_period_{sample_period_ * 5};

  /**
   * @brief The total number of landmarks in the dataset.
   */
  unsigned short int total_landmarks{};

  /**
   * @brief The total number of robots in the dataset.
   */
  unsigned short int total_robots{};

  /**
   * @brief the total number of barcodes in the dataset.
   * @note the value of this variable is the summation of the
   * DataHandler::TOTAL_LANDMARKS and DataHandler::TOTAL_ROBOTS.
   */
  unsigned short int total_barcodes_{};

  /**
   * @brief Pointer to input robot vector.
   */
  std::vector<Robot> *robots_{};

  /**
   * @brief Pointer to input landmark vector.
   */
  std::vector<Landmark> *landmarks_{};

  /**
   * @brief The simulation limits for the robots.
   * @details This is taken form the paper, "The UTIAS multi-robot cooperative
   * localization and mapping dataset". DOI: 10.1177/0278364911398404
   */
  const struct {
    double width{15.0}; ///< Maximum x-coordinate (2. Data collection: page 970)
    double height{8.0}; ///< Maximum y-coordinate (2. Data collection: page 970)
    double forward_velocity{
        0.16}; ///< Maximum forward velocity [m/s] (2.3 Odometry: page 970)
    double angular_velocity{
        0.35}; ///< Maximum angular velocity [rad/s] (2.3 Odometry: page 970)
  } limits_;

  /**
   * @brief allows for easier acces of the items in the limits_ struct.
   */
  enum Range { MIN = 0, MAX = 1 };

  struct {
    double forward_velocity[2]{0.0007, 0.0016};
    double angular_velocity[2]{0.0183, 0.0399};

    double range[2]{0.0162, 0.045};
    double bearing[2]{0.00062, 0.00596};

    double landmarks[2]{0.00004964 * 0.00004964, 0.00041465 * 0.00041465};
  } variance_;

  void assignVectorMemory();
  void setBarcodes();
  void setLandmarkPositions();
  void setErrorStatistics();
  void setRobotsInitalState();
  void setRobotOdometryAndState();
  void setRobotMeasurement();
  void setGeneratorSeed(unsigned long);
  void addGaussianNoise();
};
} // namespace Data

#endif // INCLUDE_INCLUDE_SIMULATOR_H_
