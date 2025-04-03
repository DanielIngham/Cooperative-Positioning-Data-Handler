/**
 * @file data_extractor.cpp
 * @brief Class responsible for exracting the ground-truth, odometry and measurement data from the UTIAS multi-robot localisation dataset.
 * @details The class extracts the data from the groundtruth, measurement, and populates three main 
 * @author Daniel Ingham
 * @date 2025-03-28
 */

#include "../include/data_extractor.h"

/**
 * @brief Default constructor.
 */
DataExtractor::DataExtractor(){

}
/**
 * @brief Constructor that extracts and populates class attributes using the values from the dataset provided.
 * @param[in] dataset filepath to the dataset folder.
 * @note The dataset extractor constructor only takes one dataset at at time.
 */
DataExtractor::DataExtractor(std::string dataset){
	setDataSet(dataset);

}

bool DataExtractor::readBarcodes(std::string dataset) {
	/* Check that the dataset was specified */
	if ("" == this->dataset_) {
		std::cout<< "Please specify a dataset\n";
		return false;
	}

	std::string filename = dataset + "/Barcodes.dat";
	std::ifstream file(filename);

	std::string line;

	if (file.is_open()) {
		/* Iterate through file line by line.*/
		int i = 0; 

		while (std::getline(file, line)) {
			/* Ignore file comments. */
			if (line[0] == '#') {
				continue;
			}

			/* Remove whitespaces */ 
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
			
			if (i >= TOTAL_BARCODES) {
				std::cout << "[ERROR] total read barcodes exceeds TOTAL_BARCODES\n";
				return false;
			}
			else {
				barcodes_[i++] = std::stoi(line.substr(line.find('\t', 0))) ;
			}
		}

		file.close();

		return true;
	}
	else { 
		std::cout<< "[ERROR] Unable to open barcodes file:"<< filename << std::endl;
		return false;
	}
}

bool DataExtractor::readLandmarks(std::string dataset) {

	std::string filename = dataset + "/Landmark_Groundtruth.dat";
	std::ifstream file(filename);

	std::string line;

	if (file.is_open()) {
		/* Iterate through file line by line.*/
		int i = 0; 

		while (std::getline(file, line)) {
			/* Ignore file comments. */
			if ('#' == line[0]) {
				continue;
			}

			/* Remove whitespaces */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
			
			if (i >= TOTAL_LANDMARKS) { 
				std::cout << "[ERROR] total read landmarks exceeds TOTAL_LANDMARKS\n";
				return false;
			}
			else {
				std::size_t start_index = 0; 
				std::size_t end_index = line.find('\t', 0);
				landmarks_[i].id = std::stoi(line.substr(start_index, end_index));

				if (barcodes_[landmarks_[i].id - 1] == 0) { 
					std::cout << "[ERROR] Barcodes not correctly set" << std::endl;
					return false;
				}

				landmarks_[i].barcode = barcodes_[landmarks_[i].id - 1] ;
				

				start_index = end_index; 
				end_index = line.find('\t', end_index+1);
				landmarks_[i].x = std::stod(line.substr(start_index, end_index));

				start_index = end_index; 
				end_index = line.find('\t', end_index+1);
				landmarks_[i].y = std::stod(line.substr(start_index, end_index));

				start_index = end_index; 
				end_index = line.find('\t', end_index+1);
				landmarks_[i].x_std_dev = std::stod(line.substr(start_index, end_index));

				start_index = end_index; 
				end_index = line.find('\t', end_index+1);
				landmarks_[i++].y_std_dev = std::stod(line.substr(start_index, end_index));
			}
		}

		file.close();

		return true;
	}
	else { 
		std::cout<< "[ERROR] Unable to open Landmarks file:" << filename << std::endl;
		return false;
	}
}

bool DataExtractor::readGroundTruth(std::string dataset, int robot_id) {
	/* Clear all previous elements in the ground truth vector. */
	robots_[robot_id].raw.ground_truth.clear();

	/* Setup file for data extraction */
	std::string filename = dataset + "/Robot" + std::to_string(robot_id + 1) + "_Groundtruth.dat"; 
	std::ifstream file(filename);
	std::string line;

	/* Check if the file could be opened */
	if (file.is_open()) {
		/* Loop through each line in the file. */
		while (std::getline(file, line)) {
			/* Ignore Comments */
			if ('#' == line[0]) {
				continue;
			}
			/* Remove whitespaces */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
			int start_index = 0;
			int end_index = line.find('\t', 0);
			double time = std::stod(line.substr(start_index, end_index));

			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			double x_coordinate = std::stod(line.substr(start_index, end_index));

			start_index = end_index; 
			end_index = line.find('\t', ++end_index);
			double y_coordinate = std::stod(line.substr(start_index, end_index));

			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			double orientation = std::stod(line.substr(start_index, end_index));

			robots_[robot_id].raw.ground_truth.push_back(Groundtruth(time, x_coordinate, y_coordinate, orientation));
		}
		return true;
	}
	/* If the data file could not be opened, display error. */
	else {
		std::cout<< "[ERROR] Unable to open Groundtruth file:" << filename << std::endl;
		return false;
	}
}

bool DataExtractor::readOdometry(std::string dataset, int robot_id) {
	/* Clear all previous elements in the odometry vector. */
	robots_[robot_id].raw.odometry.clear();

	/* Setup file for data extraction */
	std::string filename = dataset + "Robot" + std::to_string(robot_id + 1) +"_Odometry.dat";
	std::fstream file(filename); 
	std::string line;

	/* Check if the file could be opened */
	if (file.is_open()) {
		/* Loop through each line in the file. */
		while (std::getline(file, line)) {
			/* Ignore Comments */
			if ('#' == line[0]) {
				continue;
			}
			/* Remove Whitespace */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

			int start_index = 0;
			int end_index = line.find('\t', 0); 
			double time = std::stod(line.substr(start_index, end_index));

			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			double forward_velocity = std::stod(line.substr(start_index, end_index));
;

			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			double angular_velocity = std::stod(line.substr(start_index, end_index));
;

			robots_[robot_id].raw.odometry.push_back(Odometry(time, forward_velocity, angular_velocity));
		}

		return true;
	}
	/* If the data file could not be opened, display error. */
	else {
		std::cout<< "[ERROR] Unable to open Odometry file:" << filename << std::endl;
		return false;
	}
}

bool DataExtractor::readMeasurements(std::string dataset, int robot_id) {
	/* Clear all previous elements in the measurement vector. */
	robots_[robot_id].raw.odometry.clear();

	/* Setup file for data extraction */
	std::string filename = dataset + "/Robot" + std::to_string(robot_id+1) + "_Measurment.dat";
	std::fstream file(filename);
	std::string line;

	/* Check if the file could be opened */
	if (file.is_open()) {
		/* Loop through each line in the file. */
		while (std::getline(file, line)) {
			/* Ignore Comments */
			if ('#' == line[0]) {
				continue;
			}
			/* Remove Whitespaces */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

			int start_index = 0;
			int end_index = line.find('\t', 0);
			double time = std::stod(line.substr(start_index, end_index));
			
			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			int subject = std::stoi(line.substr(start_index, end_index));

			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			double range = std::stod(line.substr(start_index, end_index));

			start_index = end_index;
			end_index = line.find('\t', ++end_index);
			double bearing = std::stod(line.substr(start_index, end_index));

			/* Check if the current time index falls within 0.05 seconds of a previous time index. */
			auto iterator = std::find_if(robots_[robot_id].raw.measurements.begin(), robots_[robot_id].raw.measurements.end(), [&](Measurement index) {
				return index.time >= time - 0.05 && index.time <= time + 0.05;
			});

			/* If the timestamp already exists in the vector of measurements, append the current measurment to the timestamp.*/
			if (iterator != robots_[robot_id].raw.measurements.end()) {
				iterator->subjects.push_back(subject);
				iterator->ranges.push_back(range);
				iterator->bearings.push_back(bearing);
			}
			else {
				robots_[robot_id].raw.measurements.push_back(Measurement(time, subject, range, bearing));
			}
		}
		
		return true;
	}
	/* If the data file could not be opened, display error. */
	else {
		std::cout<< "[ERROR] Unable to open Measurement file:" << filename << std::endl;
		return false;
	}
}

void DataExtractor::setDataSet(std::string dataset) {
	/* Check if the data set directory exists */
	struct stat sb;
	const char* directory = dataset.c_str();

	if (stat(directory, &sb) == 0) {
		this->dataset_ = dataset;
	}
	else {
		throw std::runtime_error("Dataset file path does not exist"); 
	}
	
	/* Perform data extraction in the directory */
	bool barcodes_correct = readBarcodes(dataset);
	bool landmarks_correct = readLandmarks(dataset);
	bool ground_truth_correct = true;

	for (int i = 0; i < TOTAL_ROBOTS; i++) {
		ground_truth_correct &= readGroundTruth(dataset, i);
	}
	bool successful_extraction = barcodes_correct & landmarks_correct & ground_truth_correct;

	if (!successful_extraction) {
		throw std::runtime_error("Unable to extract data from dataset");
	}
}

int* DataExtractor::getBarcodes() {
	if ("" ==  this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data."); 
		return nullptr;
	}
	return barcodes_;
}

DataExtractor::Landmark* DataExtractor::getLandmarks() {
	if ("" ==  this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data.");
		return nullptr;
	}
	return landmarks_;
}

DataExtractor::Robot* DataExtractor::getRobots() {
	if ("" == this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data.");
		return nullptr;
	}
	return robots_;
}
