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

#include "robot.h"
#include "landmark.h"

/**
 * @class Populates Robot and Landmark vectors with simulated values.
 */
class Simulator {
public:
	Simulator(const unsigned long int, double, std::vector<Robot>&, std::vector<Landmark>&);
	Simulator(Simulator &&) = default;
	Simulator(const Simulator &) = default;
	~Simulator();

private:
	/**
	 * @ brief The total number of samples for each robot in the simulation.
	 */
	const unsigned long int data_points_ = 0;

	/**
	 * @brief The period between samples for the simulated groundtruth and odometry readings.
	 */
	double sample_period_ = 0.02;

	/**
	 * @brief Reference to input robot vector.
	 */
	std::vector<Robot>& robots_;

	/**
	 * @brief Reference to input landmark vector.
	 */
	std::vector<Landmark> landmarks_;
	
	struct Point {
		double x;
		double y;
	};
	/**
	 * @brief The simulation limits for the robots.
	 * @details This is taken form the paper, "The UTIAS multi-robot cooperative localization and mapping dataset". DOI: 10.1177/0278364911398404
	 */
	struct {
		double width = 15.0f;		///< Maximum x-coordinate (2. Data collection: page 970)
		double height = 8.0f;			///< Maximum y-coordinate (2. Data collection: page 970)
		double forward_velocity = 0.16f;	///< Maximum forward velocity [m/s] (2.3 Odometry: page 970)
		double angular_velocity = 0.35f;	///< Maximum angular velocity [rad/s] (2.3 Odometry: page 970)
	} limits_;

	void setLandmarks();

	double distance(const Point&, const Point&);
};


#endif  // INCLUDE_INCLUDE_SIMULATOR_H_
