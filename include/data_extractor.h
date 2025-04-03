/**
 * @file data_extractor.h
 * @brief Class responsible for extracting the data from the UTIAS multi-robot localisation dataset.
 * @details The class extracts the textfile dataset into three members: barcodes, landmarks, and robots. 
 * @author Your Name
 * @date YYYY-MM-DD
 */
#include <fstream>	// std::ifstream
#include <string>	// std::string
#include <algorithm>	// std::remove_if
#include <iostream>	// std::cout
#include <stdexcept>	// throw std::runtime_error
#include <sys/stat.h>	// std::stat
#include <vector>	// std::vector

#define TOTAL_LANDMARKS 15
#define TOTAL_ROBOTS 5
#define TOTAL_BARCODES (TOTAL_ROBOTS + TOTAL_LANDMARKS)

class DataExtractor {
private:
	/**
	 * @brief String of the folder location for the dataset.
	 */
	std::string dataset = "";

	/**
	 * @brief Array containing list of all barcodes corresponding to the robots and landmarks. The index of the element in the array corresponds to its id minus one.
	 * @detail Initialise all barcodes to 0. Since none of the barcodes have a value of 0, this will be used as a check to see if all the barcodes were correctly set .
	 */
	int barcodes[TOTAL_BARCODES] = {0}; 
	
	/** 
	 * @struct landmark
	 * @brief Structure containing the data attributes extracted from the dataset.
	 */
	struct Landmark { 
		int id;			///< Numerical identifier for the landmark. 
		int barcode;		///< Barcode associated with the landmark. This is what the robots will read during there operation to identify the landmarks.
		double x;		///< x-coordinate [m]
		double y;		///< y-coordinate [m]
		double x_std_dev;	///< x-standard deviation [m]
		double y_std_dev;	///< y-standard deviation [m]
	};
	
	Landmark landmarks [TOTAL_LANDMARKS];

	struct Groundtruth {
		double time;
		double x;
		double y;
		double orientation;

		double forward_velocity;
		double angular_velocity;

		double range;
		double bearing;

		Groundtruth(double time_, double x_, double y_, double orientation_): time(time_), x(x_), y(y_), orientation(orientation_) {}
	};

	struct Odometry {
		double time;
		double forward_velocity;
		double angular_velocity;
	};

	struct Measurement {
		double range;
		double bearing;
	};

	struct Robot {
		int id;			///< Numerical identifier for the robot. 
		int barcode;		///< Barcode associated with the robot. This is what the other robots will read during there operation to identify each other.
		struct {
			std::vector<Groundtruth> ground_truth;
			std::vector<Odometry> odometry;
			std::vector<Measurement> measurements;
		} raw, synced;
	};
	
	Robot robots [TOTAL_ROBOTS];
	bool readBarcodes(std::string);
	bool readLandmarks(std::string);
	bool readGroundTruth(std::string, int);
	bool readOdometry(std::string, int);
	bool readMeasurements(std::string, int);
public:
	DataExtractor(); 
	DataExtractor(std::string);

	void setDataSet(std::string);

	int* getBarcodes();
	Landmark* getLandmarks();
};
