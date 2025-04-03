#include<iostream>	// std::cout
#include<fstream>	// std::fstream
#include <string>
#include "../include/data_extractor.h"

bool check_conversion() {
	return false;
}

int main(int argc, char** argv) {
	std::cout<<"UNIT TESTING"<<std::endl;

	DataExtractor data;

	/* Loop through every MRCLAM data set and check if the file extraction was successful. */
	bool barcodes_set = true;
	bool correct_landmark_barcode = true;
	bool correct_ground_truth = true;

	for (int i = 1; i <= 9; i++) {
		std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		/* Unit Test 1: check if barcodes were set. */	
		int* barcodes = data.getBarcodes();
		for (int i = 0; i < TOTAL_BARCODES; i++) {
			if (barcodes[i] == 0) {
				barcodes_set = false; 
			}
		}
		
		/* Unit Test 2: compare landmark barcodes to barcodes. */
		auto landmarks = data.getLandmarks();
		for (int j = 0; j < TOTAL_LANDMARKS; j++) {
			if (landmarks[j].barcode != barcodes[landmarks[j].id - 1]) {
				correct_landmark_barcode = false;
			}
		}

		/* Unit Test 3: check that all the ground truth values were extracted.*/
		auto robots = data.getRobots();

		for (int k=0; k < TOTAL_ROBOTS; k++) {
			std::string groundtruth_file = dataset + "/Robot" + std::to_string(k+1) + "_Groundtruth.dat";
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
			if (robots[k].raw.ground_truth.size() != counter) {
				std::cout<< "Robot " << k + 1 << " does not have a size equal to the number of entries in the groundtruth: " << robots[k].raw.ground_truth.size() << " ≠ " << counter << std::endl;
				correct_ground_truth = false;
			}
		}

	}

	barcodes_set ? std::cout << "[PASS] All barcodes were set.\n" : std::cout << "[FAIL] All barcodes were not set.\n"  ;
	
	correct_landmark_barcode ? std::cout << "[PASS] All landmarks have the correct barcodes.\n" : std::cout << "[FAIL] Landmarks do not have the correct barcodes.\n"  ;

	correct_ground_truth ? std::cout << "[PASS] All Robots have extracted all groundtruth values from the dataset\n" : std::cout << "[FAIL] Not all robots extracted the correct number of groundtruth values from the dataset.\n";

	return 0;
}
