/**
 * @file simulator.h
 * @brief Header file of the Simulator class,
 * @author Daniel Ingham
 * @date 2025-04-25
 */
#ifndef INCLUDE_INCLUDE_SIMULATOR_H_
#define INCLUDE_INCLUDE_SIMULATOR_H_

#include <cmath>
#include <random>
#include <vector>

#include "landmark.h"
#include "robot.h"

/**
 * @class Populates Robot and Landmark vectors with simulated values.
 */
class Simulator {
public:
  Simulator();
  Simulator(const unsigned long int, double, std::vector<Robot> &,
            std::vector<Landmark> &, std::vector<unsigned short int> &);
  Simulator(Simulator &&) = default;
  Simulator(const Simulator &) = default;
  ~Simulator();

  /* Setters */
  void setSimulation(const unsigned long int, double, std::vector<Robot> &,
                     std::vector<Landmark> &,
                     std::vector<unsigned short int> &);

private:
  /**
   * @ brief The total number of samples for each robot in the simulation.
   */
  unsigned long int data_points_ = 0;

  /**
   * @brief The period between samples for the simulated groundtruth and
   * odometry readings.
   */
  double sample_period_ = 0.02;

  /**
   * @brief The total number of landmarks in the dataset.
   */
  unsigned short int total_landmarks = 0;

  /**
   * @brief The total number of robots in the dataset.
   */
  unsigned short int total_robots = 0;

  /**
   * @brief the total number of barcodes in the dataset.
   * @note the value of this variable is the summation of the
   * DataHandler::TOTAL_LANDMARKS and DataHandler::TOTAL_ROBOTS.
   */
  unsigned short int total_barcodes = 0;

  /**
   * @brief Pointer to input robot vector.
   */
  std::vector<Robot> *robots_ = nullptr;

  /**
   * @brief Pointer to input landmark vector.
   */
  std::vector<Landmark> *landmarks_ = nullptr;

  /**
   * @brief Pointer to the input barcodes vector.
   */
  std::vector<unsigned short int> *barcodes_ = nullptr;

  /**
   * @brief The simulation limits for the robots.
   * @details This is taken form the paper, "The UTIAS multi-robot cooperative
   * localization and mapping dataset". DOI: 10.1177/0278364911398404
   */
  struct {
    double width =
        15.0f; ///< Maximum x-coordinate (2. Data collection: page 970)
    double height =
        8.0f; ///< Maximum y-coordinate (2. Data collection: page 970)
    double forward_velocity =
        0.16f; ///< Maximum forward velocity [m/s] (2.3 Odometry: page 970)
    double angular_velocity =
        0.35f; ///< Maximum angular velocity [rad/s] (2.3 Odometry: page 970)
  } limits_;

  enum Range { MIN, MAX };

  struct {
    double forward_velocity[2] = {0.0007, 0.0016};
    double angular_velocity[2] = {0.0183, 0.0399};

    double range[2] = {0.0162, 0.45};
    double bearing[2] = {0.00062, 0.00596};

    double landmarks[2] = {0.00004964,
                           0.00041465}; ///< Standard Deviation of the landmarks
                                        ///< (not the variance).
  } variance;

  void assignVectorMemory();
  void setBarcodes();
  void setLandmarks();
  void setRobotsInitalState();
  void setRobotOdometry();
};

#endif // INCLUDE_INCLUDE_SIMULATOR_H_
