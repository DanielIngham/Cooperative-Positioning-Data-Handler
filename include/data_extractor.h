/**
 * @file data_extractor.h
 * @brief Class responsible for extracting the data from the UTIAS multi-robot localisation dataset.
 * @details The class extracts the textfile dataset into three members: barcodes, landmarks, and robots. 
 * @author Daniel Ingham
 * @date 2025-04-04
 */
#ifndef DATA_EXTRACTOR_H
#define DATA_EXTRACTOR_H
#include <fstream>	// std::ifstream
#include <string>	// std::string
#include <algorithm>	// std::remove_if and std::find
#include <iostream>	// std::cout
#include <stdexcept>	// throw std::runtime_error
#include <sys/stat.h>	// std::stat
#include <vector>	// std::vector
#include <cmath>	// std::floor

#define TOTAL_LANDMARKS 15
#define TOTAL_ROBOTS 5
#define TOTAL_BARCODES (TOTAL_ROBOTS + TOTAL_LANDMARKS)

/**
 * @class DataExtractor
 * @brief Extracts the data from the UTIAS Multi-robot Dataset.
 *
 */
class DataExtractor {
private:
	/**
	 * @brief Folder location for the dataset.
	 */
	std::string dataset_ = "";
	
	/**
	 * @brief  List of all barcodes corresponding to the robots and landmarks. 
	 * @note The index of the element in the array corresponds to its id minus one.
	 * @details Initialise all barcodes to 0. Since none of the barcodes have a value of 0, this will be used as a check by DataExtractor::readLandmarks to see if all the barcodes were correctly set.
	 */
	int barcodes_[TOTAL_BARCODES] = {0}; 
	
	/** 
	 * @brief Data attributes for a given landmark extracted from the dataset file.
	 */
	struct Landmark { 
		int id;			///< Numerical identifier for the landmark. 
		int barcode;		///< Barcode associated with the landmark. This is what the robots will read during there operation to identify the landmarks.
		double x;		///< The landmark's golbal x-coordinate [m]
		double y;		///< The landmark's golbal y-coordinate [m]
		double x_std_dev;	///< The x-standard deviation of the positioning error [m]
		double y_std_dev;	///< The y-standard deviation of the positioning error [m]
	};
	
	/**
	 * @brief All landmarks containing all the data extracted form Landmarks.dat. 
	 */
	Landmark landmarks_[TOTAL_LANDMARKS];

	/**
	 * @brief Data attributes for a single groundtruth reading extracted from Robotx_Groundtruth.dat.
	 * @note According the UTIAS website, the groundtruth readings are accurate to the order of 1mm (1E^-3 m). Additionally, the NTP deamon used to synchronise the clocks between robots has an average timing error of 1ms (1E-3 s).
	 */
	struct Groundtruth {
		/* The following attributes are extracted directly from the robots groundtruth datafile. */
		double time;			///< Time stamp of the ground truth reading [s].
		double x;			///< Robot Groundtruth x-coordinate [m].
		double y;			///< Robot Groundtruth y-coordinate [m].
		double orientation;		///< Robot Groundtruth orientation [rad].

		/* The following attributes are calculated from the extracted attributes above. */
		double forward_velocity;
		double angular_velocity;

		double range;
		double bearing;

		Groundtruth();
		/** 
		 * @brief Constructor for convenient population of DataExtractor::robots_.raw.ground_truth .
		 */
		Groundtruth(double time_, double x_, double y_, double orientation_): time(time_), x(x_), y(y_), orientation(orientation_) {}
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
	 * @brief All data related to a given robot. 
	 */
	struct Robot {
		int id;			///< Numerical identifier for the robot. 
		int barcode;		///< Barcode associated with the robot. This is what the other robots will read during there operation to identify each other.

		/**
		 * @brief struct containing vectors related to the robot data files.
		 * @details The struct has two instances: raw and sync. The raw instance is the data from the dataset extracted as is. The synce sychronises the time steps of each robot. 
		 */
		struct {
			std::vector<Groundtruth> ground_truth;	///< All groundtruth values extracted for the given robot.
			std::vector<Odometry> odometry;		///< All odometry inputs extracted for the given robot.
			std::vector<Measurement> measurements;	///< All measurements taken by the given robot.
		} raw, synced;
	};
	
	/**
	 * @brief  All data for all robots.
	 */
	Robot robots_[TOTAL_ROBOTS];

	bool readBarcodes(const std::string&);
	bool readLandmarks(const std::string&);
	bool readGroundTruth(const std::string&, int);
	bool readOdometry(const std::string&, int);
	bool readMeasurements(const std::string&, int);

	void syncData(const double&);

	void calculateGroundtruthOdometry();
public:
	DataExtractor(); 
	explicit DataExtractor(const std::string&, const double& sampling_period = 0.02);

	void setDataSet(const std::string&, const double& sampling_period = 0.02);

	int* getBarcodes();
	Landmark* getLandmarks();
	Robot* getRobots();

};
#endif
