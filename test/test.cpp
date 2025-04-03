#include<iostream>	// std::cout
#include<fstream>	// std::fstream
#include<string>	// std::string
#include<thread>	// std::thread
#include<atomic>	// std::atomic
#include<chrono>

#include "../include/data_extractor.h"

#define TOTAL_DATASETS 9


/* Unit Test 1: check if barcodes were set. */	
void checkBarcodes(bool& barcodes_set) {
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		/* Unit Test 1: check if barcodes were set. */	
		int* barcodes = data.getBarcodes();
		for (int i = 0; i < TOTAL_BARCODES; i++) {
			if (barcodes[i] == 0) {
				barcodes_set = false; 
			}
		}
	}
	return;
}

/* Unit Test 2: compare landmark barcodes to barcodes. */
void checkLandmarkBarcodes(bool& correct_landmark_barcode) {
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		int* barcodes = data.getBarcodes();
		auto landmarks = data.getLandmarks();

		for (int j = 0; j < TOTAL_LANDMARKS; j++) {
			if (landmarks[j].barcode != barcodes[landmarks[j].id - 1]) {
				correct_landmark_barcode = false;
			}
		}
	}
	return;
}

/* Unit Test 3: check that all the ground truth values were extracted.*/
void checkGroundtruthExtraction( std::atomic<bool>& correct_ground_truth, int robot_id) {
	DataExtractor data;

	for (int i=1; i <= TOTAL_DATASETS; i++) {
		std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		auto robots = data.getRobots();

		std::string groundtruth_file = dataset + "/Robot" + std::to_string(robot_id+1) + "_Groundtruth.dat";
		std::ifstream file(groundtruth_file);
		std::string line;

		int counter = 0;
		if (file.is_open()) {
			while(std::getline(file, line)) {
				if ('#' == line[0]) {
					continue;
				}
				counter++;
			}
		}
		else {
			std::cout << "[Error] Failed to open groundtruth file:" << groundtruth_file << std::endl;
			correct_ground_truth = false;
		}
		if (robots[robot_id].raw.ground_truth.size() != counter) {
			std::cout<< "Robot " << robot_id + 1 << " does not have a size equal to the number of entries in the groundtruth: " << robots[robot_id].raw.ground_truth.size() << " ≠ " << counter << std::endl;
			correct_ground_truth = false;
		}
	} 

	return;
}


int main(int argc, char** argv) {
	auto start = std::chrono::high_resolution_clock::now();
	std::cout<< "UNIT TESTING" <<std::endl;
	std::cout<< "Number of treads supported: " << std::thread::hardware_concurrency() << std::endl;

	/* Loop through every MRCLAM data set and check if the file extraction was successful. */
	bool barcodes_set = true;
	bool correct_landmark_barcode = true;
	std::atomic<bool> correct_ground_truth(true);
	
	std::thread unit_test_1(checkBarcodes, std::ref(barcodes_set));
	std::thread unit_test_2(checkLandmarkBarcodes, std::ref(correct_landmark_barcode));
	std::thread unit_test_3[TOTAL_ROBOTS];

	for (int i = 0; i < TOTAL_ROBOTS; i++) {
		unit_test_3[i] = std::thread(checkGroundtruthExtraction, std::ref(correct_ground_truth), i);
	}

	unit_test_1.join();
	unit_test_2.join();
	for (int j = 0; j < TOTAL_ROBOTS; j++) {
		unit_test_3[j].join();
	}
	
	barcodes_set ? std::cout << "[PASS] All barcodes were set.\n" : std::cout << "[FAIL] All barcodes were not set.\n"  ;
	
	correct_landmark_barcode ? std::cout << "[PASS] All landmarks have the correct barcodes.\n" : std::cout << "[FAIL] Landmarks do not have the correct barcodes.\n"  ;

	correct_ground_truth ? std::cout << "[PASS] All Robots have extracted all groundtruth values from the dataset\n" : std::cout << "[FAIL] Not all robots extracted the correct number of groundtruth values from the dataset.\n";

	auto end = std::chrono::high_resolution_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::seconds>(end-start);

	std::cout << "\n Test ran for: " << duration.count() << " seconds\n";
	return 0;
}
