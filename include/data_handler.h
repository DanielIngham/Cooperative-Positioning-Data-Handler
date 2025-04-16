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

	unsigned short int TOTAL_BARCODES = 0;
	unsigned short int TOTAL_LANDMARKS = 0;
	unsigned short int TOTAL_ROBOTS = 0;
	/**
	 * @brief  List of all barcodes corresponding to the robots and landmarks. 
	 * @note The index of the element in the array corresponds to its id minus one.
	 * @details Initialise all barcodes to 0. Since none of the barcodes have a value of 0, this will be used as a check by DataHandler::readLandmarks to see if all the barcodes were correctly set.
	 */
	std::vector<int> barcodes_; 
	
	/**
	 * @brief All landmarks containing all the data extracted form Landmarks.dat. 
	 */
	std::vector<Landmark> landmarks_;

	/**
	 * @brief  All data for all robots.
	 */
	std::vector<Robot> robots_;

	bool readBarcodes(const std::string&);
	bool readLandmarks(const std::string&);
	bool readGroundTruth(const std::string&, int);
	bool readOdometry(const std::string&, int);
	bool readMeasurements(const std::string&, int);

	void syncData(const double&);

	void calculateGroundtruthOdometry();
	void calculateGroundtruthMeasurement();


	void saveStateData(bool&);
	void saveOdometryData(bool&);
	void saveMeasurementData(bool&);
	void saveErrorData(bool&);
	void saveOdometryErrorPDF(bool&);
	void saveMeasurementErrorPDF(bool&);

	void relativeRobotDistance();
	void relativeLandmarkDistance();
public:
	DataHandler(); 
	explicit DataHandler(const std::string&, const double& sampling_period = 0.02);

	void setDataSet(const std::string&, const double& sampling_period = 0.02);

	std::vector<int>& getBarcodes();
	std::vector<Landmark>& getLandmarks();
	std::vector<Robot>& getRobots();

	double getSamplePeriod();

	unsigned short int getNumberOfRobots();
	unsigned short int getNumberOfLandmarks();
	unsigned short int getNumberOfBarcodes();

	int getID(int);
	void saveData(bool&);

};

#endif  // INCLUDE_INCLUDE_DATA_EXTRACTOR_H_
