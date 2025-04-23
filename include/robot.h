/**
 * @file robot.h
 * @brief Header file of the Robot class.
 * @author Daniel Ingham
 * @date 2025-04-16
 */
#ifndef INCLUDE_INCLUDE_ROBOT_H_
#define INCLUDE_INCLUDE_ROBOT_H_

#include <vector>	// std::vector
#include <stdexcept>	// std::runtime_error
#include <string>	// std::string
#include <iostream>	// std::cout
#include <numeric>	// std::accumulate
#include <cmath>	// std::atan2
#include <numeric>	// std::accumulate

/**
 * @class Robot
 * @brief Houses all data and functionality related to a given robot in a multi-robot localisation environment
 */
class Robot {
private:
public:
	Robot();
	Robot(Robot &&) = default;
	Robot(const Robot &) = default;
	Robot &operator=(Robot &&) = default;
	Robot &operator=(const Robot &) = default;
	~Robot();

	int id;			///< Numerical identifier for the robot. 
	int barcode;		///< Barcode associated with the robot. This is what the other robots will read during there operation to identify each other.

	/**
	 * @brief Data attributes for a single groundtruth reading extracted from Robotx_Groundtruth.dat.
	 * @note According the UTIAS website, the groundtruth readings are accurate to the order of 1mm (1E^-3 m). Additionally, the NTP deamon used to synchronise the clocks between robots has an average timing error of 1ms (1E-3 seconds).
	 */
	struct State {
		/* The following attributes are extracted directly from the robots groundtruth datafile. */
		double time;			///< Time stamp of the ground truth reading [s].

		double x;			///< Robot Groundtruth x-coordinate [m].
		double y;			///< Robot Groundtruth y-coordinate [m].
		double orientation;		///< Robot Groundtruth orientation [rad].
		 
		State();
		/** 
		 * @brief Constructor for convenient population of DataExtractor::robots_.raw.ground_truth .
		 */
		State(double time_, double x_, double y_, double orientation_): time(time_), x(x_), y(y_), orientation(orientation_) {}
	};

	/**
	 * @brief Data attributes of a single odometry reading extracted from Robotx_Odometry.dat.
	 */
	struct Odometry {
		double time;			///< Time stamp of the odometry readings [s].
		double forward_velocity;	///< The robots forward velocity [m/s].
		double angular_velocity;	///< The robots angular velocity [rad/s].

		Odometry();
		/** 
		 * @brief Constructor for convenient population of DataExtractor::robots_.raw.odometry .
		 */
		Odometry(double time_, double forward_velocity_, double angular_velocity_): time(time_), forward_velocity(forward_velocity_), angular_velocity(angular_velocity_) {} ;
	};

	/**
	 * @brief Data attributes of the relative distance measurement taken at a single timestep in Robotx_Measurements.
	 * @note This structure combines multiple measurements with the same time stamp. There it is assumed that the vectors: subjects, ranges, and bearings are in the same order.
	 */
	struct Measurement {
		double time;			///< Time stamp of the measurement [s].
		std::vector<int> subjects;	///< The Barcode of the other robots being measured.
		std::vector<double> ranges;	///< The measured ranges to the subjects [m]
		std::vector<double> bearings;	///< The bearings from the subjects [rad]

		Measurement(double time_,  std::vector<int> subjects_, std::vector<double> ranges_, std::vector<double> bearings_): time(time_), subjects(subjects_), ranges(ranges_), bearings(bearings_) {};
		/** 
		 * @brief Constructor for convenient population of DataExtractor::robots_.raw.measurements .
		 */
		Measurement(double time_,  int subject_, double range_, double bearing_): time(time_){
			subjects.push_back(subject_);
			ranges.push_back(range_);
			bearings.push_back(bearing_);
		};
	};

	/**
	 * @brief Vectors related to the robot data. 
	 */
	struct {
		std::vector<State> states;		///< All groundtruth values extracted for the given robot.
		std::vector<Odometry> odometry;		///< All odometry inputs extracted for the given robot.
		std::vector<Measurement> measurements;	///< All measurements taken by the given robot.
	} raw,		///< The raw data extracted from the dataset's .dat files. 
	synced,		///< The odometry and measurement values with synced timesteps.
	groundtruth,	///< The groundtruth
	error;		///< The difference between the ground truth and the synced data


	/**
	 * @brief Error statistics used by filters for inference.
	 * @details It is often assumed that all errors are caussed by white Gaussian distributed noise, which forms the foundation for the devolopment of Bayesian filtering. For this reason, the robots noise statistics are calculated.
	 */
	struct {
		double mean = 0.0;		///< The sample mean of the error. 
		double variance = 0.0;///< The sample standard deviation of the error. 
	} range_error,		///< Error associated with the range measurements.
	bearing_error,		///< Error associated with the bearing measurements.
	forward_velocity_error,	///< Error associated with the forward velocity input.
	angular_velocity_error;	///< Error associated with the angular velocity input.

	void calculateMeasurementError();

	void calculateSampleErrorStats();


};

#endif  // INCLUDE_INCLUDE_ROBOT_H_
