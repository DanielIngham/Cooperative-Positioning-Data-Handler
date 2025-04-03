#include<iostream>
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

	for (int i = 1; i <= 9; i++) {
		data.setDataSet("./data/MRCLAM_Dataset" + std::to_string(i));

		int* barcodes = data.getBarcodes();
		auto landmarks = data.getLandmarks();
		
		/* Unit Test 1 */	
		for (int i = 0; i < TOTAL_BARCODES; i++) {
			if (barcodes[i] == 0) {
				barcodes_set = false; 
			}
		}
		
		/* Unit Test 2 */
		for (int j = 0; j < TOTAL_LANDMARKS; j++) {
			if (landmarks[j].barcode != barcodes[landmarks[j].id - 1]) {
				correct_landmark_barcode = false;
			}
		}
	}

	barcodes_set ? std::cout << "[PASS] All barcodes were set.\n" : std::cout << "[FAIL] All barcodes were not set.\n"  ;
	
	correct_landmark_barcode ? std::cout << "[PASS] All landmarks have the correct barcodes.\n" : std::cout << "[FAIL] Landmarks do not have the correct barcodes.\n"  ;

	return 0;
}
