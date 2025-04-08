#include <cstddef>
#include <cstdint>
#include <iostream>	// std::cout
#include <fstream>	// std::fstream
#include <string>	// std::string
#include <thread>	// std::thread
#include <chrono>	// std::chrono
#include <algorithm>	// std::find

#include "../include/data_extractor.h"

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
/**
 * @brief Saves the data from the DataExtractor class into .dat files to be plotted by gnuplot.
 */
void savePlotData(bool& flag) {
	DataExtractor data;

	data.setDataSet("./data/MRCLAM_Dataset1");
	const auto* robots = data.getRobots();

	std::ofstream robot_file;

	/* Loop through the data structures for each robot */
	for (std::uint8_t i = 0; i < TOTAL_ROBOTS; i++ ) {
		/* Save the values of the raw and synced groundtruth values of a given robot into the same file with the last row indicating 'r' for raw and 's' for synced.*/
		std::string filename = "./test/data/robot" + std::to_string(i) + "-Groundtruth" + ".dat";
		robot_file.open(filename);
		if (!robot_file.is_open()) {
			std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
			flag = false;
			return;
		}

		for (std::size_t j = 0; j < robots[i].raw.ground_truth.size(); j++) {
			robot_file << robots[i].raw.ground_truth[j].time << '\t' << robots[i].raw.ground_truth[j].x << '\t' << robots[i].raw.ground_truth[j].y << '\t' << robots[i].raw.ground_truth[j].orientation << '\t' << 'r' << '\n';
			
			if (j < robots[i].synced.ground_truth.size()){
				robot_file << robots[i].synced.ground_truth[j].time << '\t' << robots[i].synced.ground_truth[j].x << '\t' << robots[i].synced.ground_truth[j].y << '\t'<< robots[i].synced.ground_truth[j].orientation << '\t' << 's' << '\n';
			}
		}

		/* Save the values of the raw and synced odometry values of a given robot into the same file with the last row indicating 'r' for raw  and 's' for synced.*/
		robot_file.close();
		filename = "./test/data/robot" + std::to_string(i) + "-Odometry" + ".dat";
		robot_file.open(filename);

		if (!robot_file.is_open()) {
			std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
			flag = false;
			return;
		}

		for (std::size_t j = 0; j < robots[i].raw.odometry.size(); j++) {
			robot_file << robots[i].raw.odometry[j].time << '\t' << robots[i].raw.odometry[j].forward_velocity << '\t' << robots[i].raw.odometry[j].angular_velocity << '\t' << 'r' << '\n';
			
			if (j < robots[i].synced.odometry.size()){
				robot_file << robots[i].synced.odometry[j].time << '\t' << robots[i].synced.odometry[j].forward_velocity << '\t' << robots[i].synced.odometry[j].angular_velocity << '\t' << 's' << '\n';
			}
		}

		/* Save the values of the raw and synced measurment values of a given robot into the same file with the last row indicating 'g' for raw  and 'i' for synced.*/
		robot_file.close();
		filename = "./test/data/robot" + std::to_string(i) + "-Meaurement" + ".dat";
		robot_file.open(filename);

		if (!robot_file.is_open()) {
			std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
			flag = false;
			return;
		}

		/* Note that when the "raw" measurement data structure is populated, it only adds one element to the members for each time stamp. After interpolation, these values are combined if they have the same time stamp.*/
		for (std::size_t j = 0; j < robots[i].raw.measurements.size(); j++) {
			robot_file << robots[i].raw.measurements[j].time << '\t' << robots[i].raw.measurements[j].subjects[0] << '\t' << robots[i].raw.measurements[j].ranges[0] << '\t' <<  robots[i].raw.measurements[j].bearings[0] << '\t' << 'r' << '\n';
		}
		for (std::size_t j = 0; j < robots[i].synced.measurements.size(); j++) {
			for (std::size_t k = 0; k < robots[i].synced.measurements[j].subjects.size(); k++) {
				robot_file << robots[i].synced.measurements[j].time << '\t' << robots[i].synced.measurements[j].subjects[k] << '\t' << robots[i].synced.measurements[j].ranges[k] << '\t' << robots[i].synced.measurements[j].bearings[k] << '\t' << 's' << '\n';
			}
		}
		robot_file.close();
	}
}

/**
 * @brief Unit Test 1: check if barcodes were set.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void checkBarcodes(bool& flag) {
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		/* Unit Test 1: check if barcodes were set. */	
		const int* barcodes = data.getBarcodes();
		for (int j = 0; j < TOTAL_BARCODES; j++) {
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
	DataExtractor data;

	for (int i = 1; i <= TOTAL_DATASETS; i++) {
		const std::string dataset = "./data/MRCLAM_Dataset" + std::to_string(i);
		data.setDataSet(dataset);

		const int* barcodes = data.getBarcodes();
		const auto* landmarks = data.getLandmarks();

		for (int j = 0; j < TOTAL_LANDMARKS; j++) {
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
 * @param [in,out] flag indicates whether the test was passed or failed.
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
 * @param [in,out] flag indicates whether the test was passed or failed.
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

/**
 * @brief Unit Test 6: Test Interpolation values against the ones extracted from the matlab script.
 * @param [in,out] flag indicates whether the test was passed or failed.
 */
void testInterpolation(bool& flag) { 
	DataExtractor data("./data/MRCLAM_Dataset1");
	auto* robots = data.getRobots();

	for (std::uint8_t k = 0; k < TOTAL_ROBOTS; k++) {

		/* Check Groundtruth Interpolation  */
		std::string filename = "./test/Matlab_output/Robot" + std::to_string(k+ 1) + "_Groundtruth.csv";
		std::fstream file(filename);

		if (!file.is_open()) {
			std::cerr << "Unable to open file: " << filename << std::endl;
			flag = false;
			return;
		}

		/* Check that the number of lines in the file match the number of items in the extracted values. */
		std::size_t total_lines = countFileLines(filename);
		if (total_lines != robots[k].synced.ground_truth.size()) {
			std::cerr << "Total number of interpolated groundtruth values does not match Matlab output " << robots[k].synced.ground_truth.size() << " ≠ " << total_lines <<". File: " << filename << std::endl;
			flag = false;
			return;
		}

		for (std::size_t i = 0; i < robots[k].synced.ground_truth.size(); i++) {
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

			if (std::round((robots[k].synced.ground_truth[i].time - time )* 100.0f)/ 100.0f  !=  0 ) {
				std::cerr << "Interpolation Time Index Error [Line " << i << "] " << filename << ": " << robots[k].synced.ground_truth[i].time << " ≠ " << time << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double x_coordinate = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[k].synced.ground_truth[i].x - x_coordinate) * 100.0f)/ 100.0f != 0) {
				std::cerr << "Interpolation x-coordinate Error [Line " << i << "] " << filename << ": " << robots[k].synced.ground_truth[i].x << " ≠ " << x_coordinate << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double y_coordinate = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[k].synced.ground_truth[i].y - y_coordinate ) * 100.0f)/ 100.0f != 0) {
				std::cerr << "Interpolation y-coordinate Error [Line " << i << "] " << filename << ": "<< robots[k].synced.ground_truth[i].y << " ≠ " << y_coordinate << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double orientation = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[k].synced.ground_truth[i].orientation - orientation) * 100.0f) / 100.0 != 0) {
				std::cerr << "Interpolation orientation Error [Line " << i << "] " << filename << ":" << robots[k].synced.ground_truth[i].orientation << " ≠ " << orientation << std::endl;
				flag = false;
				return;
			}
		}

		/* Check Odometry Interpolation */
		file.close();
		filename = "./test/Matlab_output/Robot" + std::to_string(k + 1) + "_Odometry.csv";
		file.open(filename);
		if (!file.is_open()) {
			std::cerr << "Unable to open file: " << filename << std::endl;
			flag = false;
			return;
		}

		/* Check that the number of lines in the file match the number of items in the extracted values. */
		total_lines = countFileLines(filename);
		if (total_lines != robots[k].synced.odometry.size()) {
			std::cerr << "Total number of interpolated Odometry values does not match Matlab output " << robots[k].synced.odometry.size() << " ≠ " << total_lines << " . File: " << filename;
			flag = false;
			return;
		}

		for (std::size_t i = 0; i < robots[k].synced.odometry.size(); i++) {
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

			if (std::round((robots[k].synced.odometry[i].time - time )* 100.0f)/ 100.0f  !=  0 ) {
				std::cout << "Interpolation Time Index Error [Line " << i << "] " << filename << ": " << robots[k].synced.odometry[i].time << " ≠ " << time << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double forward_velocity = std::stod(line.substr(start_index, end_index - start_index));
			if (std::round((robots[k].synced.odometry[i].forward_velocity - forward_velocity) * 100.0f)/ 100.0f != 0) {
				std::cerr << "Interpolation forward velocity Error [Line " << i << "] " << filename << ": " << robots[k].synced.odometry[i].forward_velocity << " ≠ " << forward_velocity << std::endl;
				flag = false;
				return;
			}

			start_index = end_index + 1;
			end_index = line.find(',', start_index);
			double angular_velocity = std::stod(line.substr(start_index, end_index - start_index));

			if (std::round((robots[k].synced.odometry[i].angular_velocity - angular_velocity ) * 100.0f)/ 100.0f != 0) {
				std::cerr << "Interpolation angular velocity Error [Line " << i << "] " << filename << ": " << robots[k].synced.odometry[i].angular_velocity << " ≠ " << angular_velocity << std::endl;
				flag = false;
				return;
			}
		}

		/* Check Measurement Interpolation  */
		file.close();
		filename = "./test/Matlab_output/Robot" + std::to_string(k + 1) + "_Measurement.csv";
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
		for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
			/* Check all the measurement vectors are the same length */ 
			if ((robots[k].synced.measurements[i].subjects.size() != robots[k].synced.measurements[i].ranges.size()) || (robots[k].synced.measurements[i].ranges.size() != robots[k].synced.measurements[i].bearings.size())) {
				std::cerr << "Measurement size mismatch: subjects, ranges, and bearings do not have the same size: " << robots[k].synced.measurements[i].subjects.size() << " : " << robots[k].synced.measurements[i].ranges.size() << " : " << robots[k].synced.measurements[i].bearings.size() << "\n"; 
				flag = false; 
				return; 
			} 

			total_measurements += robots[k].synced.measurements[i].subjects.size();
		}

		if (total_lines != total_measurements) {
			std::cerr << "Total number of interpolated Measurment values does not match Matlab output " << total_measurements << " ≠ " << total_lines << filename <<  std::endl;
			flag = false;
			return;
		}

		std::string line;
		std::size_t counter = 0;
		double current_time = -1.0f;

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
			if (current_time == -1.0f) {
				current_time = time;
			}
			else if (time > current_time) {
				/* Perform checking on previously populated list */
				if (std::round((robots[k].synced.measurements[counter].time - current_time ) * 100.0f)/ 100.0f  !=  0 ) {
					std::cerr << "Interpolation Time Index Error [Line " << counter << "] ./test/Matlab_output/Robot1_Measurement.csv: " << robots[k].synced.measurements[counter].time << " ≠ " << time << std::endl;
					flag = false;
					return;
				}

				if (robots[k].synced.measurements[counter].subjects != subjects) {
					std::cerr << "List of subjects does not match\n"; 
					std::cerr << current_time << std::endl;
					std::cerr << robots[k].synced.measurements[counter].subjects.size() << " : " << subjects.size() << std::endl;
					for (std::uint8_t i = 0; i < robots[k].synced.measurements[counter].subjects.size(); i++) {
						std::cout << robots[k].synced.measurements[counter].subjects[i] << '\t';
					}
					std::cout << '\n';

					for (std::uint8_t i = 0; i < subjects.size(); i++) {
						std::cout << subjects[i] << '\t';
					}
					std::cout << "\n\n";

					flag = false;
					return;
				}

				if (robots[k].synced.measurements[counter].ranges != ranges) {
					std::cerr << "List of ranges does not match\n"; 
					flag = false;
					return;
				}

				if (robots[k].synced.measurements[counter].bearings != bearings) {
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
	DataExtractor data("./data/MRCLAM_Dataset1");

	auto* robots = data.getRobots();
	double sample_period = data.getSamplePeriod();

	for (int k = 0; k < TOTAL_ROBOTS; k++) {
		/* Check if the synced groundtruth and odometry are the same length */
		if (robots[k].synced.ground_truth.size() != robots[k].synced.odometry.size()) {
			std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " groundtruth and odometry vectors are not the same length\n";
			flag = false;
			return;
		}

		/* Check if all the sample periods are equal to the dataExtractor::sample_period_ */
		for (std::size_t i = 1; i < robots[k].synced.ground_truth.size(); i++) {
			double extracted_sample_period = (robots[k].synced.ground_truth[i].time - robots[k].synced.ground_truth[i-1].time);

			if (std::round((extracted_sample_period - sample_period)*1000.0f) / 1000.f != 0) {
				std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " synced groundtruth time stamps do not matching sampling period:" << extracted_sample_period << " ≠ " << sample_period << std::endl;
				flag = false;
				return;
			}
		}

		for (std::size_t i = 1; i < robots[k].synced.odometry.size(); i++) {
			double extracted_sample_period = (robots[k].synced.odometry[i].time - robots[k].synced.odometry[i-1].time);

			if (std::round((extracted_sample_period - sample_period)*1000.0f) / 1000.f != 0) {
				std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " synced odometry time stamps do not matching sampling period:" << extracted_sample_period << " ≠ " << sample_period << std::endl;
				flag = false;
				return;
			}
		}

		/* Check if all the sample time stamps are the same between Groundtruth and odometry.
		 * NOTE: The sizes of the measurements and groundtruth vectors are checked above, so they are assumed to be equal here. 
		 */ 
		for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
			if (robots[k].synced.odometry[i].time != robots[k].synced.ground_truth[i].time) {
			std::cerr << "\033[1;31m[ERROR]\033[0m Robot " << k << " Time stamp mismatch between odometry and groundtruth: " << robots[k].synced.odometry[i].time << " ≠ " << robots[k].synced.ground_truth[i].time << std::endl;
			}
		}
		/* Check if all the measurements have the time stamps as Groundruth. */
		auto iterator = robots[k].synced.ground_truth.begin();
		for (std::size_t i = 0; i < robots[k].synced.measurements.size(); i++) {
			/* Attempt to find the timestep in the ground truth time steps. 
			 * NOTE: since the groundtruth and odometry has already been checked to be the same, it is assumed that if the time stamp is in the groundtruth, it is also in odometry. */
			iterator = std::find_if(iterator, robots[k].synced.ground_truth.end(), [&](const auto& element) {
				return (std::round((element.time - robots[k].synced.measurements[i].time) * 1000.0f) / 1000.0f == 0.0f);
			});
			/* If the measurement time stamp is not in the ground, an error has occured. */
			if (iterator == robots[k].synced.ground_truth.end()) {
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
// void testGroundtruthOdometry(bool& flag) {
// 	DataExtractor data("./data/MRCLAM_Dataset1");
//
// 	auto* robots = data.getRobots();
//
// 	for (std::size_t i = 0; i < robots->synced.ground_truth.size(); i++) {
// 		double delta_t = 0;
// 		double x = robots->synced.ground_truth[i].x + robots->synced.ground_truth[i].forward_velocity ;
// 		flag = false;
// 	}
//
// }

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
	bool plot_data_saved = true;

	std::thread unit_test_1(checkBarcodes, std::ref(barcodes_set));
	std::thread unit_test_2(checkLandmarkBarcodes, std::ref(correct_landmark_barcode));
	std::thread unit_test_3(checkGroundtruthExtraction, std::ref(correct_groundtruth));
	std::thread unit_test_4(checkOdometryExtraction, std::ref(correct_odometry));
	std::thread unit_test_5(checkMeasurementExtraction, std::ref(correct_measurements));
	std::thread unit_test_6(testInterpolation, std::ref(correct_interpolation));
	std::thread unit_test_7(checkSamplingRate, std::ref(correct_sampling_rate));
	std::thread unit_test_8(savePlotData, std::ref(plot_data_saved));

	unit_test_1.join();
	unit_test_2.join();
	unit_test_3.join();
	unit_test_4.join();
	unit_test_5.join();
	unit_test_6.join();
	unit_test_7.join();
	unit_test_8.join();

	barcodes_set ? std::cout << "\033[1;32m[U1 PASS]\033[0m All barcodes were set.\n" : std::cerr << "[U1 FAIL] All barcodes were not set.\n"  ;

	correct_landmark_barcode ? std::cout << "\033[1;32m[U2 PASS]\033[0m All landmarks have the correct barcodes.\n" : std::cerr << "\033[1;31m[U2 FAIL]\033[0m Landmarks do not have the correct barcodes.\n"  ;

	correct_groundtruth ? std::cout << "\033[1;32m[U3 PASS]\033[0m All Robots have extracted the correct amount groundtruth values from the dataset\n" : std::cerr << "\033[1;31m[U3 FAIL]\033[0m Not all robots extracted the correct amount of groundtruth values from the dataset.\n";

	correct_odometry ? std::cout << "\033[1;32m[U4 PASS]\033[0m All Robots have extracted the correct amount of odometry values from the dataset\n" : std::cerr << "\033[1;31m[U3 FAIL]\033[0m Not all robots extracted the correct amount of odometery values from the dataset.\n";

	correct_measurements ? std::cout << "\033[1;32m[U5 PASS]\033[0m All Robots have extracted the correct amount of measurement values from the dataset\n" : std::cerr << "\033[1;31m[U5 FAIL]\033[0m Not all robots extracted the correct amount of measurement values from the dataset.\n";

	correct_interpolation ? std::cout << "\033[1;32m[U6 PASS]\033[0m All raw extracted values were correctly interpolated\n" : std::cerr << "\033[1;31m[U6 FAIL]\033[0m Raw extraced values were not correctly interpolated\n";
	
	correct_sampling_rate ? std:: cout << "\033[1;32m[U7 PASS]\033[0m All resampled data have the same time stamps \n" : std::cerr << "\033[1;31m[U7 FAIL] The timesteps in the synced datasets did not match.\n";

	plot_data_saved ? std::cout << "\033[1;32m[U8 PASS]\033[0m Plot saved.\n" : std::cerr << "\033[1;32m[U8 FAIL]\033[0m Failed to save plot.\n";
	
	// bool correctGroundruthOdometry = true;
	// testGroundtruthOdometry(correctGroundruthOdometry);

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(end-start);
	std::cout << "\n Test ran for: " << duration.count() << " seconds\n";
	return 0;
}
