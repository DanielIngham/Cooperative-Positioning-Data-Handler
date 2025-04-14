/**
 * @file data_extractor.h
 * @brief Class responsible for extracting the data from the UTIAS multi-robot localisation dataset.
 * @details The class extracts the textfile dataset into three members: barcodes, landmarks, and robots. 
 * @author Daniel Ingham
 * @date 2025-04-04
 */
#ifndef INCLUDE_INCLUDE_DATA_HANDLER_H_
#define INCLUDE_INCLUDE_DATA_HANDLER_H_

#include <fstream>	// std::ifstream
#include <string>	// std::string
#include <algorithm>	// std::remove_if and std::find
#include <iostream>	// std::cout
#include <stdexcept>	// throw std::runtime_error
#include <sys/stat.h>	// std::stat
#include <vector>	// std::vector
#include <cmath>	// std::floor
#include <unordered_map>// std::unordered_map
#include <filesystem>	// std::filesystem

#include "../include/robot.h"
#include "../include/landmark.h"

#define TOTAL_LANDMARKS 15
#define TOTAL_ROBOTS 5
#define TOTAL_BARCODES (TOTAL_ROBOTS + TOTAL_LANDMARKS)
#define TOTAL_DATASETS 9

/**
 * @class DataHandler
 * @brief Extracts the data from the UTIAS Multi-robot Dataset.
 */
class DataHandler {
private:
	/**
	 * @brief Folder location for the dataset.
	 */
	std::string dataset_ = "";

	/**
	 * @brief Folder location for the output data.
	 */
	std::string output_directory_ = "";
	
	/**
	 * @brief the desired sample period for resampling the data to sync the timesteps between the vehicles.
	 */
	double sampling_period_ = 0.2;

	/**
	 * @brief  List of all barcodes corresponding to the robots and landmarks. 
	 * @note The index of the element in the array corresponds to its id minus one.
	 * @details Initialise all barcodes to 0. Since none of the barcodes have a value of 0, this will be used as a check by DataHandler::readLandmarks to see if all the barcodes were correctly set.
	 */
	int barcodes_[TOTAL_BARCODES] = {0}; 
	
	/**
	 * @brief All landmarks containing all the data extracted form Landmarks.dat. 
	 */
	Landmark landmarks_[TOTAL_LANDMARKS];

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
	void calculateGroundtruthMeasurement();

	int getID(int);

	void saveStateData(bool&);
	void saveOdometryData(bool&);
	void saveMeasurementData(bool&);
	void saveErrorData(bool&);
	void saveOdometryErrorPDF(bool&);

	void relativeRobotDistance();
	void relativeLandmarkDistance();
public:
	DataHandler(); 
	explicit DataHandler(const std::string&, const double& sampling_period = 0.02);

	void setDataSet(const std::string&, const double& sampling_period = 0.02);

	int* getBarcodes();
	Landmark* getLandmarks();
	Robot* getRobots();
	double getSamplePeriod();

	void saveData(bool&);
};

#endif  // INCLUDE_INCLUDE_DATA_EXTRACTOR_H_
