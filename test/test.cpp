#include<iostream>	// std::cout
#include<fstream>	// std::fstream
#include<string>	// std::string
#include<thread>	// std::thread
#include<atomic>	// std::atomic
#include<chrono>

#include "../include/data_extractor.h"

#define TOTAL_DATASETS 9

/**
 * @brief Loops through the entire dataset file and counts the number of lines that are not commented.
 * @return the number of lines counted.
 */
long unsigned int countFileLines(const std::string& filename) {
	std::ifstream file(filename);

	if (file.is_open()) {
		long unsigned int counter = 0;
		std::string line;
		while(std::getline(file, line)) {
			if ('#' == line[0]) {
				continue;
			}
			counter++;
		}
		return counter;
	}
	else {
		std::cout << "[Error] Failed to open file:" << filename << std::endl;
		return 0;
	}
}

/**
 * @brief Unit Test 1: check if barcodes were set.
 */
void checkBarcodes(bool& barcodes_set) {
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		/* Unit Test 1: check if barcodes were set. */	
		const int* barcodes = data.getBarcodes();
		for (int j = 0; j < TOTAL_BARCODES; j++) {
			if (barcodes[j] == 0) {
				barcodes_set = false; 
			}
		}
	}
	return;
}

/** 
 * @brief Unit Test 2: compare landmark barcodes to barcodes. 
 */
void checkLandmarkBarcodes(bool& correct_landmark_barcode) {
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		const int* barcodes = data.getBarcodes();
		const auto* landmarks = data.getLandmarks();

		for (int j = 0; j < TOTAL_LANDMARKS; j++) {
			if (landmarks[j].barcode != barcodes[landmarks[j].id - 1]) {
				correct_landmark_barcode = false;
			}
		}
	}
	return;
}

/** 
 * @brief Unit Test 3: check that all the ground truth values were extracted.
 */
void checkGroundtruthExtraction(bool& flag) {
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);

		data.setDataSet(dataset);

		const auto* robots = data.getRobots();

		for (int robot_id = 0; robot_id < TOTAL_ROBOTS; robot_id++) {
			std::string groundtruth_file = dataset + "/Robot" + std::to_string(robot_id+1) + "_Groundtruth.dat";

			long unsigned int counter = countFileLines(groundtruth_file);

			if (0 == counter) {
				flag = false;
			}

			else if (robots[robot_id].raw.ground_truth.size() != counter) {
				std::cout<< "Robot " << robot_id + 1 << " does not have a size equal to the number of entries in the groundtruth: " << robots[robot_id].raw.ground_truth.size() << " ≠ " << counter << std::endl;
				flag = false;
			}
		}
	} 
	return;
}

/** 
 * @brief Unit Test 4: check that all the odometry values were extracted.
 */
void checkOdometryExtraction(bool& flag) {
	DataExtractor data;

	/* Loop through every data */
	for (int i = 1; i <= TOTAL_DATASETS; i++) {

		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);

		data.setDataSet(dataset);

		const auto* robots = data.getRobots();

		/* Loop through every robot */
		for (int robot_id = 0; robot_id < TOTAL_ROBOTS; robot_id++) {

			std::string odometry_file = dataset + "/Robot" + std::to_string(robot_id+1) + "_Odometry.dat";

			long unsigned int counter = countFileLines(odometry_file);

			if (counter == 0) {
				flag = false;
			}
			else if (robots[robot_id].raw.odometry.size() != counter) {
				std::cout<< "Robot " << robot_id + 1 << " does not have a size equal to the number of entries in the odometry file: " << robots[robot_id].raw.odometry.size() << " ≠ " << counter << std::endl;
				flag = false;
			}
		}
	} 
	return;
}

/** 
 * @brief Unit Test 5: check that all the measurement values were extracted.
 */
void checkMeasurementExtraction(bool& flag) {
	DataExtractor data;

	/* Loop through every data */
	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		const auto* robots = data.getRobots();

		/* Loop through every robot */
		for (int robot_id = 0; robot_id < TOTAL_ROBOTS; robot_id++) {
			std::string measurement_file = dataset + "/Robot" + std::to_string(robot_id+1) + "_Measurement.dat";

			long unsigned int counter = countFileLines(measurement_file);

			if (0 == counter) {
				flag = false;
			}

			/* Count the number of elements */ 
			long unsigned int measurement_counter = 0;

			for (std::size_t j = 0; j < robots[robot_id].raw.measurements.size(); j++) {
				if ((robots[robot_id].raw.measurements[j].bearings.size() == robots[robot_id].raw.measurements[j].ranges.size()) && (robots[robot_id].raw.measurements[j].ranges.size() == robots[robot_id].raw.measurements[j].subjects.size())) {
					measurement_counter += robots[robot_id].raw.measurements[j].subjects.size();
				}
				else {
					flag = false;
				}
			}
			if (measurement_counter != counter) {
				std::cout<< "Robot " << robot_id + 1 << " does not have a size equal to the number of entries in the measurement file: " << robots[robot_id].raw.measurements.size() << " ≠ " << counter << std::endl;
				flag = false;
			}
		}
	} 
	return;
}

int main() {
	auto start = std::chrono::high_resolution_clock::now();
	std::cout<< "UNIT TESTING" <<std::endl;
	std::cout<< "Number of treads supported: " << std::thread::hardware_concurrency() << std::endl;

	// /* Loop through every MRCLAM data set and check if the file extraction was successful. */
	bool barcodes_set = true;
	bool correct_landmark_barcode = true;
	bool correct_groundtruth = true;
	bool correct_odometry = true;
	bool correct_measurements = true;
	
	std::thread unit_test_1(checkBarcodes, std::ref(barcodes_set));
	std::thread unit_test_2(checkLandmarkBarcodes, std::ref(correct_landmark_barcode));
	std::thread unit_test_3(checkGroundtruthExtraction, std::ref(correct_groundtruth));
	std::thread unit_test_4(checkOdometryExtraction, std::ref(correct_odometry));
	std::thread unit_test_5(checkMeasurementExtraction, std::ref(correct_measurements));

	unit_test_1.join();
	unit_test_2.join();
	unit_test_3.join();
	unit_test_4.join();
	unit_test_5.join();

	barcodes_set ? std::cout << "[PASS] All barcodes were set.\n" : std::cout << "[FAIL] All barcodes were not set.\n"  ;

	correct_landmark_barcode ? std::cout << "[PASS] All landmarks have the correct barcodes.\n" : std::cout << "[FAIL] Landmarks do not have the correct barcodes.\n"  ;

	correct_groundtruth ? std::cout << "[PASS] All Robots have extracted the correct amount groundtruth values from the dataset\n" : std::cout << "[FAIL] Not all robots extracted the correct amount of groundtruth values from the dataset.\n";

	correct_odometry ? std::cout << "[PASS] All Robots have extracted the correct amount of odometry values from the dataset\n" : std::cout << "[FAIL] Not all robots extracted the correct amount of odometery values from the dataset.\n";

	correct_measurements ? std::cout << "[PASS] All Robots have extracted the correct amount of measurement values from the dataset\n" : std::cout << "[FAIL] Not all robots extracted the correct amount of measurement values from the dataset.\n";

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(end-start);
	std::cout << "\n Test ran for: " << duration.count() << " seconds\n";
	return 0;
}
