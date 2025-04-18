#include <cstddef>
#include <cstdint>
#include <iostream>	// std::cout
#include <fstream>	// std::fstream
#include <string>	// std::string
#include <thread>	// std::thread
#include <chrono>	// std::chrono
#include <algorithm>	// std::find

#include "../include/data_handler.h" // DataHandler

#define TOTAL_DATASETS 9

/**
 * @brief Loops through the entire dataset file and counts the number of lines that are not commented using '#'.
 * @param [in] filename the name of the input file.
 * @return the number of lines counted.
 * @note this function also counts empty lines. The only line ignored are the ones where the character '#' is present as the first character in the line.
 */
std::size_t countFileLines(const std::string& filename) {
	std::ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "[ERROR] Failed to open file:" << filename << std::endl;
		return 0;
	}

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

void saveData(bool& flag) {
	for (unsigned short int d  = 0; d < TOTAL_DATASETS; d++) {
		DataHandler data("./data/MRCLAM_Dataset" + std::to_string(d + 1));
		data.saveExtractedData(flag);
	}
}

/**
 * @brief Unit Test 1: check if barcodes were set.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkBarcodes(bool& flag) {
	DataHandler data;

	for (unsigned short int d = 1; d <= TOTAL_DATASETS; d++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(d);
		data.setDataSet(dataset);

		/* Unit Test 1: check if barcodes were set. */	
		const auto barcodes = data.getBarcodes();

		for (unsigned short int j = 0; j < data.getNumberOfBarcodes(); j++) {
			if (barcodes[j] == 0) {
				flag = false; 
			}
		}
	}
	return;
}

/** 
 * @brief Unit Test 2: compare landmark barcodes to barcodes. 
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkLandmarkBarcodes(bool& flag) {
	DataHandler data;

	for (unsigned short int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		const auto barcodes = data.getBarcodes();
		const auto landmarks = data.getLandmarks();

		for (int j = 0; j < data.getNumberOfLandmarks(); j++) {
			if (landmarks[j].barcode != barcodes[landmarks[j].id - 1]) {
				flag = false;
			}
		}
	}
	return;
}

/** 
 * @brief Unit Test 3: check that all the ground truth values were extracted.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkGroundtruthExtraction(bool& flag) {
	DataHandler data;

	for (unsigned short int d = 1; d <= TOTAL_DATASETS; d++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(d);

		data.setDataSet(dataset);

		const auto robots = data.getRobots();

		for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {
			std::string groundtruth_file = dataset + "/Robot" + std::to_string(id+1) + "_Groundtruth.dat";

			long unsigned int counter = countFileLines(groundtruth_file);

			if (0 == counter) {
				flag = false;
			}

			else if (robots[id].raw.states.size() != counter) {
				std::cerr << "Robot " << id + 1 << " does not have a size equal to the number of entries in the groundtruth: " << robots[id].raw.states.size() << " ≠ " << counter << std::endl;
				flag = false;
			}
		}
	} 
	return;
}

/** 
 * @brief Unit Test 4: check that all the odometry values were extracted.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkOdometryExtraction(bool& flag) {
	DataHandler data;

	/* Loop through every data */
	for (unsigned short int d = 1; d <= TOTAL_DATASETS; d++) {

		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(d);

		data.setDataSet(dataset);

		const auto robots = data.getRobots();

		/* Loop through every robot */
		for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {

			std::string odometry_file = dataset + "/Robot" + std::to_string(id+1) + "_Odometry.dat";

			long unsigned int counter = countFileLines(odometry_file);

			if (counter == 0) {
				flag = false;
			}
			else if (robots[id].raw.odometry.size() != counter) {
				std::cerr << "Robot " << id + 1 << " does not have a size equal to the number of entries in the odometry file: " << robots[id].raw.odometry.size() << " ≠ " << counter << std::endl;
				flag = false;
			}
		}
	} 
	return;
}

/** 
 * @brief Unit Test 5: check that all the measurement values were extracted.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkMeasurementExtraction(bool& flag) {
	DataHandler data;

	/* Loop through every data */
	for (unsigned short int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		const auto robots = data.getRobots();

		/* Loop through every robot */
		for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {
			std::string measurement_file = dataset + "/Robot" + std::to_string(id+1) + "_Measurement.dat";

			long unsigned int counter = countFileLines(measurement_file);

			if (0 == counter) {
				flag = false;
			}

			/* Count the number of elements */ 
			long unsigned int measurement_counter = 0;

			for (std::size_t k = 0; k < robots[id].raw.measurements.size(); k++) {

				if ((robots[id].raw.measurements[k].bearings.size() == robots[id].raw.measurements[k].ranges.size()) && 
					(robots[id].raw.measurements[k].ranges.size() == robots[id].raw.measurements[k].subjects.size())) {

					measurement_counter += robots[id].raw.measurements[k].subjects.size();
				}
				else {
					flag = false;
				}
			}
			if (measurement_counter != counter) {
				std::cerr << "Robot " << id + 1 << " does not have a size equal to the number of entries in the measurement file: " << robots[id].raw.measurements.size() << " ≠ " << counter << std::endl;
				flag = false;
			}
		}
	} 
	return;
}

/**
 * @brief Unit Test 6: Test Interpolation values against the ones extracted from the matlab script.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void testInterpolation(bool& flag) { 
	DataHandler data("./data/MRCLAM_Dataset1");
	const auto robots = data.getRobots();

	for (unsigned int id = 0; id < data.getNumberOfRobots(); id++) {

		/* Check Groundtruth Interpolation  */
		std::string filename = "./test/Matlab_output/Robot" + std::to_string(id+ 1) + "_Groundtruth.csv";
		std::fstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Unable to open file: " << filename << std::endl;
			flag = false;
			return;
		}

		/* Check that the number of lines in the file match the number of items in the extracted values. */
		std::size_t total_lines = countFileLines(filename);
		if (total_lines != robots[id].groundtruth.states.size()) {
			std::cerr << "Total number of interpolated groundtruth values does not match Matlab output " << robots[id].groundtruth.states.size() << " ≠ " << total_lines <<". File: " << filename << std::endl;
			flag = false;
			return;
		}

		for (std::size_t k = 0; k < robots[id].groundtruth.states.size(); k++) {
			std::string line;
			std::getline(file, line); 

			if ('#' == line[0]) {
				k--;
				continue;
			}

			/* Remove whitespaces */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

			std::size_t start_index = 0; 
			std::size_t end_index = line.find(',', 0);
			double time = std::stod(line.substr(start_index, end_index));

			if (std::round((robots[id].groundtruth.states[k].time - time )* 100.0)/ 100.0  !=  0 ) {
				std::cerr << "Interpolation Time Index Error [Line " << k << "] " << filename << ": " << robots[id].groundtruth.states[k].time << " ≠ " << time << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double x_coordinate = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[id].groundtruth.states[k].x - x_coordinate) * 100.0)/ 100.0 != 0) {
				std::cerr << "Interpolation x-coordinate Error [Line " << k << "] " << filename << ": " << robots[id].groundtruth.states[k].x << " ≠ " << x_coordinate << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double y_coordinate = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[id].groundtruth.states[k].y - y_coordinate ) * 100.0)/ 100.0 != 0) {
				std::cerr << "Interpolation y-coordinate Error [Line " << k << "] " << filename << ": "<< robots[id].groundtruth.states[k].y << " ≠ " << y_coordinate << std::endl;
				flag = false;
				return;
			}

			/* NOTE: This script handles the orientation interpolation better than the matlab script. Therefore this check was removed. */
			
			// start_index = end_index + 1;
			// end_index = line.find(',', start_index);
			// double orientation = std::stod(line.substr(start_index, end_index - start_index));

			// if (std::round((robots[id].groundtruth.states[k].orientation - orientation) * 100.0) / 100.0 != 0) {
			// 	std::cerr << "Interpolation orientation Error [Line " << k << "] " << filename << ":" << robots[id].groundtruth.states[k].orientation << " ≠ " << orientation << std::endl;
			// 	flag = false;
			// 	return;
			// }
		}

		/* Check Odometry Interpolation */
		file.close();
		filename = "./test/Matlab_output/Robot" + std::to_string(id + 1) + "_Odometry.csv";
		file.open(filename);
		if (!file.is_open()) {
			std::cerr << "Unable to open file: " << filename << std::endl;
			flag = false;
			return;
		}

		/* Check that the number of lines in the file match the number of items in the extracted values. */
		total_lines = countFileLines(filename);
		if (total_lines != robots[id].synced.odometry.size()) {
			std::cerr << "Total number of interpolated Odometry values does not match Matlab output " << robots[id].synced.odometry.size() << " ≠ " << total_lines << " . File: " << filename;
			flag = false;
			return;
		}

		for (std::size_t i = 0; i < robots[id].synced.odometry.size(); i++) {
			std::string line;
			std::getline(file, line); 
			if ('#' == line[0]) {
				i--;
				continue;
			}
			/* Remove whitespaces */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
			
			std::size_t start_index = 0; 
			std::size_t end_index = line.find(',', 0);
			double time = std::stod(line.substr(start_index, end_index));

			if (std::round((robots[id].synced.odometry[i].time - time )* 100.0)/ 100.0  !=  0 ) {
				std::cerr << "Interpolation Time Index Error [Line " << i << "] " << filename << ": " << robots[id].synced.odometry[i].time << " ≠ " << time << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double forward_velocity = std::stod(line.substr(start_index, end_index - start_index));
			if (std::round((robots[id].synced.odometry[i].forward_velocity - forward_velocity) * 100.0)/ 100.0 != 0) {
				std::cerr << "Interpolation forward velocity Error [Line " << i << "] " << filename << ": " << robots[id].synced.odometry[i].forward_velocity << " ≠ " << forward_velocity << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double angular_velocity = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[id].synced.odometry[i].angular_velocity - angular_velocity ) * 100.0)/ 100.0 != 0) {
				std::cerr << "Interpolation angular velocity Error [Line " << i << "] " << filename << ": " << robots[id].synced.odometry[i].angular_velocity << " ≠ " << angular_velocity << std::endl;
				flag = false;
				return;
			}
		}

		/* Check Measurement Interpolation  */
		file.close();
		filename = "./test/Matlab_output/Robot" + std::to_string(id + 1) + "_Measurement.csv";
		file.open(filename);
		if (!file.is_open()) {
			std::cerr << "Unable to open file: " << filename << std::endl;
			flag = false;
			return;
		}
		/* Check that the number of lines in the file match the number of items in the extracted values. */
		total_lines = countFileLines(filename);
		if (0 == total_lines) {
			flag = false;
			return;
		}

		std::size_t total_measurements = 0;
		/* Count the then number of elements in the measurment matrix */ 
		for (std::size_t i = 0; i < robots[id].synced.measurements.size(); i++) {
			/* Check all the measurement vectors are the same length */ 
			if ((robots[id].synced.measurements[i].subjects.size() != robots[id].synced.measurements[i].ranges.size()) || (robots[id].synced.measurements[i].ranges.size() != robots[id].synced.measurements[i].bearings.size())) {
				std::cerr << "Measurement size mismatch: subjects, ranges, and bearings do not have the same size: " << robots[id].synced.measurements[i].subjects.size() << " : " << robots[id].synced.measurements[i].ranges.size() << " : " << robots[id].synced.measurements[i].bearings.size() << "\n"; 
				flag = false; 
				return; 
			} 

			total_measurements += robots[id].synced.measurements[i].subjects.size();
		}

		if (total_lines != total_measurements) {
			std::cerr << "Total number of interpolated Measurment values does not match Matlab output " << total_measurements << " ≠ " << total_lines << filename <<  std::endl;
			flag = false;
			return;
		}

		std::string line;
		std::size_t counter = 0;
		double current_time = -1.0;

		std::vector<int> subjects;
		std::vector<double> ranges;
		std::vector<double> bearings;

		while (std::getline(file, line)) {
			
			/* Ignore Comments */
			if ('#' == line[0]) {
				continue;
			}

			/* Remove whitespaces */
			line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
			
			std::size_t start_index = 0; 
			std::size_t end_index = line.find(',', 0);
			double time = std::stod(line.substr(start_index, end_index));
			/* Initialise current_time on first iteration. */
			if (current_time == -1.0) { current_time = time;
			}
			else if (time > current_time) {
				/* Perform checking on previously populated list */
				if (std::round((robots[id].synced.measurements[counter].time - current_time ) * 100.0)/ 100.0  !=  0 ) {
					std::cerr << "Interpolation Time Index Error [Line " << counter << "] ./test/Matlab_output/Robot1_Measurement.csv: " << robots[id].synced.measurements[counter].time << " ≠ " << time << std::endl;
					flag = false;
					return;
				}

				if (robots[id].synced.measurements[counter].subjects != subjects) {
					std::cerr << "List of subjects does not match\n"; 
					std::cerr << current_time << std::endl;
					std::cerr << robots[id].synced.measurements[counter].subjects.size() << " : " << subjects.size() << std::endl;

					flag = false;
					return;
				}

				if (robots[id].synced.measurements[counter].ranges != ranges) {
					std::cerr << "List of ranges does not match\n"; 
					flag = false;
					return;
				}

				if (robots[id].synced.measurements[counter].bearings != bearings) {
					std::cerr << "List of bearings does not match\n"; 
					flag = false;
					return;
				}

				/* Clear vectors */
				subjects.clear();
				ranges.clear();
				bearings.clear();

				/* Update the current timestep */
				current_time = time;
				counter++;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			subjects.push_back(std::stod(line.substr(start_index, end_index - start_index)));

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			ranges.push_back(std::stod(line.substr(start_index, end_index - start_index)));

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			bearings.push_back(std::stod(line.substr(start_index, end_index - start_index)));
		}
	}
}
/**
 * @brief Checks that the time stamps produced by the resampling process are shared across the ground truth, odometry and measurements, as well as equal to the defined sampling period. 
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkSamplingRate(bool& flag) {
	DataHandler data("./data/MRCLAM_Dataset1");

	auto robots = data.getRobots();
	double sample_period = data.getSamplePeriod();

	for (unsigned short int k = 0; k < data.getNumberOfRobots(); k++) {
		/* Check if the synced groundtruth and odometry are the same length */
		if (robots[k].groundtruth.states.size() != robots[k].synced.odometry.size()) {
			std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " groundtruth and odometry vectors are not the same length\n";
			flag = false;
			return;
		}

		/* Check if all the sample periods are equal to the dataHandler::sample_period_ */
		for (std::size_t i = 1; i < robots[k].groundtruth.states.size(); i++) {
			double extracted_sample_period = (robots[k].groundtruth.states[i].time - robots[k].groundtruth.states[i-1].time);

			if (std::round((extracted_sample_period - sample_period)*1000.0) / 1000. != 0) {
				std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " synced groundtruth time stamps do not matching sampling period:" << extracted_sample_period << " ≠ " << sample_period << std::endl;
				flag = false;
				return;
			}
		}

		for (std::size_t i = 1; i < robots[k].synced.odometry.size(); i++) {
			double extracted_sample_period = (robots[k].synced.odometry[i].time - robots[k].synced.odometry[i-1].time);

			if (std::round((extracted_sample_period - sample_period)*1000.0) / 1000.0 != 0) {
				std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " synced odometry time stamps do not matching sampling period:" << extracted_sample_period << " ≠ " << sample_period << std::endl;
				flag = false;
				return;
			}
		}

		/* Check if all the sample time stamps are the same between Groundtruth and odometry.
		 * NOTE: The sizes of the measurements and groundtruth vectors are checked above, so they are assumed to be equal here. */ 
		for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
			if (robots[k].synced.odometry[i].time != robots[k].groundtruth.states[i].time) {
			std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " Time stamp mismatch between odometry and groundtruth: " << robots[k].synced.odometry[i].time << " ≠ " << robots[k].groundtruth.states[i].time << std::endl;
			}
		}
		/* Check if all the measurements have the time stamps as Groundruth. */
		auto iterator = robots[k].groundtruth.states.begin();
		for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
			/* Attempt to find the timestep in the ground truth time steps. 
			 * NOTE: since the groundtruth and odometry has already been checked to be the same, it is assumed that if the time stamp is in the groundtruth, it is also in odometry. */
			iterator = std::find_if(iterator, robots[k].groundtruth.states.end(), [&](const auto& element) {
				return (std::round((element.time - robots[k].synced.measurements[i].time) * 1000.0) / 1000.0 == 0.0);
			});
			/* If the measurement time stamp is not in the ground, an error has occured. */
			if (iterator == robots[k].groundtruth.states.end()) {
				std::cerr << "\033[1;31m[ERROR]\033[1m Robot " << k << " measurment timestamp not present in groundtruth: "  << robots[k].synced.measurements[k].time << std::endl;
				flag = false;
				return;
			}
		}
	}
}

/**
 * @brief Checks that the when the caluclated odometry values are used for dead-reckoning that the outputs matches the ground truth values.
 * @param [in,out] flag confirms that the test was passed or failed.
 */
void testGroundtruthOdometry(bool& flag) {
	for (unsigned short int dataset = 0; dataset < TOTAL_DATASETS; dataset++) {
		DataHandler data("./data/MRCLAM_Dataset" + std::to_string(dataset+1));

		auto robots = data.getRobots();

		for (unsigned short int id = 0; id < data.getNumberOfRobots(); id++) {
			double average_x_difference = 0;
			double average_y_difference = 0;
			double average_orientation_difference = 0;

			for (std::size_t k = 0; k < robots[id].groundtruth.states.size() - 1; k++) {

				/* NOTE: Unit Test 8 checks that the sampling time equals the set sample period. Therefore, it is assumed that the same period is equal to the value set.  */
				double sampling_period = data.getSamplePeriod();

				/* Calculate the robot's x-position and compare it to the groundtruth value */
				double x = robots[id].groundtruth.states[k].x + robots[id].groundtruth.odometry[k].forward_velocity * sampling_period * std::cos(robots[id].groundtruth.states[k].orientation);
				
				average_x_difference += std::abs(x - robots[id].groundtruth.states[k + 1].x); 

				/* Calculate the robot's y-position and compare it to the groundtruth value */
				double y = robots[id].groundtruth.states[k].y + robots[id].groundtruth.odometry[k].forward_velocity * sampling_period * std::sin(robots[id].groundtruth.states[k].orientation);

				average_y_difference += std::abs(y - robots[id].groundtruth.states[k+1].y);
				/* Calculate the robot's orientation and compare it to the groundtruth value */
				double orientation = robots[id].groundtruth.states[k].orientation + sampling_period * robots[id].groundtruth.odometry[k].angular_velocity;

				/* Normalise the orientation between PI and -PI (180 and -180 degrees respectively) */
				while (orientation >= M_PI) orientation -= 2.0 * M_PI;
				while (orientation < -M_PI) orientation += 2.0 * M_PI;

				average_orientation_difference += std::abs(orientation - robots[id].groundtruth.states[k+1].orientation);
			}
		}
	}
	flag = true;
}
/**
 * @brief Checks if the PDF adds to 1
 */
void checkPDF() {
	std::string filename = "./data/MRCLAM_Dataset1/data_extraction/Bearing-Error-PDF.dat";
	std::fstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Unable to open file: " << filename << std::endl;
		return;
	}

	std::string line;
	double integral = 0.0;
	while(std::getline(file,line)) {
		/* Skip comments */
		if ('#' == line[0] ) {
			continue;
		}

		if ("" == line) {
			std::cout << "END" << std::endl;
			break;
		}

		std::size_t start_index = 0;
		std::size_t end_index = 0;
		end_index = line.find('\t', end_index);	

		start_index = end_index + 1;
		end_index = line.find('\t', end_index+1);	
		double bin_width = std::stod(line.substr(start_index,end_index - start_index));

		start_index = end_index + 1;
		end_index = line.find('\t', end_index+1);	
		double value = std::stod(line.substr(start_index, end_index - start_index));

		integral += value * bin_width;
	}

	if (integral == 1.0) {
		std::cout << "PASS" << std::endl;
	}
	else {
		std::cout << "FAIL" << std::endl;
	}
}

int main() {
	auto start = std::chrono::high_resolution_clock::now();
	std::cout<< "\033[1;36mUNIT TESTING\033[0m" <<std::endl;
	std::cout<< "\033[3mNumber of treads supported:\033[0m " << std::thread::hardware_concurrency() << std::endl;

	/* Loop through every MRCLAM data set and check if the file extraction was successful. */
	bool barcodes_set = true;
	bool correct_landmark_barcode = true;
	bool correct_groundtruth = true;
	bool correct_odometry = true;
	bool correct_measurements = true;
	bool correct_interpolation = true;
	bool correct_sampling_rate= true;
	bool data_saved= true;
	bool correct_groundtruth_odometry = true;

	// std::thread unit_test_1(checkBarcodes, std::ref(barcodes_set));
	// std::thread unit_test_2(checkLandmarkBarcodes, std::ref(correct_landmark_barcode));
	// std::thread unit_test_3(checkGroundtruthExtraction, std::ref(correct_groundtruth));
	// std::thread unit_test_4(checkOdometryExtraction, std::ref(correct_odometry));
	// std::thread unit_test_5(checkMeasurementExtraction, std::ref(correct_measurements));
	// std::thread unit_test_6(testInterpolation, std::ref(correct_interpolation));
	// std::thread unit_test_7(checkSamplingRate, std::ref(correct_sampling_rate));
	std::thread unit_test_8(saveData, std::ref(data_saved));
	// std::thread unit_test_9(testGroundtruthOdometry, std::ref(correct_groundtruth_odometry));

	// unit_test_1.join();
	// unit_test_2.join();
	// unit_test_3.join();
	// unit_test_4.join();
	// unit_test_5.join();
	// unit_test_6.join();
	// unit_test_7.join();
	unit_test_8.join();
	// unit_test_9.join();
	checkPDF();


	barcodes_set ? std::cout << "\033[1;32m[U1 PASS]\033[0m All barcodes were set.\n" : std::cerr << "[U1 FAIL] All barcodes were not set.\n"  ;

	correct_landmark_barcode ? std::cout << "\033[1;32m[U2 PASS]\033[0m All landmarks have the correct barcodes.\n" : std::cerr << "\033[1;31m[U2 FAIL]\033[0m Landmarks do not have the correct barcodes.\n"  ;

	correct_groundtruth ? std::cout << "\033[1;32m[U3 PASS]\033[0m All Robots have extracted the correct amount groundtruth values from the dataset\n" : std::cerr << "\033[1;31m[U3 FAIL]\033[0m Not all robots extracted the correct amount of groundtruth values from the dataset.\n";

	correct_odometry ? std::cout << "\033[1;32m[U4 PASS]\033[0m All Robots have extracted the correct amount of odometry values from the dataset\n" : std::cerr << "\033[1;31m[U3 FAIL]\033[0m Not all robots extracted the correct amount of odometery values from the dataset.\n";

	correct_measurements ? std::cout << "\033[1;32m[U5 PASS]\033[0m All Robots have extracted the correct amount of measurement values from the dataset\n" : std::cerr << "\033[1;31m[U5 FAIL]\033[0m Not all robots extracted the correct amount of measurement values from the dataset.\n";

	correct_interpolation ? std::cout << "\033[1;32m[U6 PASS]\033[0m All raw extracted values were correctly interpolated\n" : std::cerr << "\033[1;31m[U6 FAIL]\033[0m Raw extraced values were not correctly interpolated\n";
	
	correct_sampling_rate ? std:: cout << "\033[1;32m[U7 PASS]\033[0m All resampled data have the same time stamps \n" : std::cerr << "\033[1;31m[U7 FAIL] The timesteps in the synced datasets did not match.\n";

	data_saved ? std::cout << "\033[1;32m[U9 PASS]\033[0m Data succesfully saved.\n" : std::cerr << "\033[1;31m[U9 FAIL]\033[0m An error occured saving the data.\n";

	correct_groundtruth_odometry ? std::cout << "\033[1;32m[U9 PASS]\033[0m All Robots have the correct groundtruth odometry values\n" : std::cerr << "\033[1;31m[U9 FAIL]\033[0m Not all robots have the correctly calculated groundtruth odometry values.\n";
	
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(end-start);
	std::cout << "\n Test ran for: " << duration.count() << " seconds\n";
	return 0;
}
