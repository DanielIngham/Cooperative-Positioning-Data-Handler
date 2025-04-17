/**
 * The class extracts the data from the groundtruth states, measured range and bearings, and odometry. These values are used to populate three main data structs: Barcodes, Landmarks, and Robots. 
 * @file data_extractor.cpp
 * @brief Class implementation file responsible for exracting the ground-truth, odometry and measurement data from the UTIAS multi-robot localisation dataset.
 * @author Daniel Ingham
 * @date 2025-03-28
 */

#include "../include/data_handler.h"
#include <stdexcept>

/**
 * @brief Default constructor.
 */
DataHandler::DataHandler(){

}
/**
 * @brief Constructor that extracts and populates class attributes using the values from the dataset provided.
 * @param[in] dataset directory path to the dataset folder.
 * @param[in] sample_period the desired sample period for resampling the data to sync the timesteps between the vehicles.
 * @note The dataset extractor constructor only takes one dataset at at time.
 */
DataHandler::DataHandler(const std::string& dataset,const double& sample_period): TOTAL_BARCODES(20), TOTAL_LANDMARKS(15), TOTAL_ROBOTS(5), barcodes_(TOTAL_BARCODES, 0), landmarks_(TOTAL_LANDMARKS), robots_(TOTAL_ROBOTS) {
	setDataSet(dataset, sample_period);
}

/**
 * @brief Extracts data from the all files in the specified dataset folder.
 * @param[in] dataset dataset path to the dataset folder.
 * @param[in] sample_period the desired sample period for resampling the data to sync the timesteps between the vehicles.
 * @note All datasets in the UTIAS Multi-robot Localisation and mapping dataset have the same number of landmarks and robots. Therefore, if the dataset directory is provided, it is assumed that the number of landmarks and robots will be 5 and 15 respectively. The number of barcodes is the summation and these two values. 
 * @note The function only checks the existence of the given datset folder. The data extraction is performed by calling the functions: DataHandler::readBarcodes, DataHandler::readLandmarks, DataHandler::readGroundTruth, DataHandler::readOdometry, and DataHandler::readMeasurements. Additionally, the DataHandler::syncData function is called to resample to data points through linear interpolation to ensure all robots have the same time stamps.
 */
void DataHandler::setDataSet(const std::string& dataset, const double& sample_period) {
	/* Check if the data set directory exists */
	struct stat sb;
	const char* directory = dataset.c_str();

	if (stat(directory, &sb) == 0) {
		this->dataset_ = dataset;
	}
	else {
		throw std::runtime_error("Dataset file path does not exist: " + dataset); 
	}

	/* Set the sample period for this dataset. */
	this->sampling_period_ = sample_period;

	this->TOTAL_BARCODES = 20U; 
	this->TOTAL_LANDMARKS = 15U; 
	this->TOTAL_ROBOTS = 5U; 

	/* Resize the dataset vectors */
	this->barcodes_.resize(TOTAL_BARCODES, 0); 
	this->landmarks_.resize(TOTAL_LANDMARKS); 
	this->robots_.resize(TOTAL_ROBOTS);

	/* Perform data extraction in the directory */
	bool barcodes_correct = readBarcodes(dataset);
	bool landmarks_correct = readLandmarks(dataset);
	bool groundtruth_correct = true;
	bool odometry_correct = true;
	bool measurement_correct = true;

	/* Populate the values for each robot from the dataset */
	for (int i = 0; i < TOTAL_ROBOTS; i++) {
		groundtruth_correct &= readGroundTruth(dataset, i);
		odometry_correct &= readOdometry(dataset, i);
		measurement_correct &= readMeasurements(dataset, i);
	}

	/* Checks that the setting of all data attributes have been succesful */
	bool successful_extraction = barcodes_correct & landmarks_correct & groundtruth_correct & odometry_correct & measurement_correct;

	if (!successful_extraction) {
		throw std::runtime_error("Unable to extract data from dataset");
	}

	/* Perform Time Stamp Synchronisation. This performs the linear interpolations of the values — ensuring all values have the same time steps  */
	syncData(sample_period);

	/* Calculate the odometry values that would correspond to the ground truth position and heading values after synchronsation. */
	calculateGroundtruthOdometry();

	/* Calculate the measurement values that would correspond to the ground truth range and bearing values. */
	calculateGroundtruthMeasurement();

	/* Calculate odometry and measurement errors. */
	for (int i = 0; i < TOTAL_ROBOTS; i++) {
		robots_[i].calculateMeasurementError();
	}
}

/**
 * @brief Extracts data from the barcodes data file: Barcodes.dat.
 * @param[in] directory path to the dataset folder.
 * @return a flag indicating whether the barcodes were succesfully extracted from the dataset.
 */
bool DataHandler::readBarcodes(const std::string& dataset) {
	/* Check that the dataset was specified */
	if ("" == this->dataset_) {
		std::cerr << "Please specify a dataset\n";
		return false;
	}

	std::string filename = dataset + "/Barcodes.dat";
	std::ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "[ERROR] Unable to open barcodes file:"<< filename << std::endl;
		return false;
	}

	/* Iterate through file line by line.*/
	std::string line;
	int i = 0; 

	while (std::getline(file, line)) {
		/* Ignore file comments. */
		if (line[0] == '#') {
			continue;
		}

		/* Remove whitespaces */ 
		line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
		
		if (i >= TOTAL_BARCODES) {
			std::cerr << "[ERROR] The number of barcodes read exceeds total number of barcodes specified." << std::endl;
			return false;
		}

		if (barcodes_.size() == 0) {
			std::cerr << "\033[1;32m[ERROR]\033[0m The total number of barcodes was not specified." << std::endl;
		}

		/* Extract barcodes into barcodes array */
		barcodes_[i++] = std::stoi(line.substr(line.find('\t', 0)));
	}

	file.close();

	return true;
}
/**
 * @brief Extracts data from the landmarks data file: Landmark_Groundtruth.dat.
 * @param[in] dataset path to the dataset folder.
 * @return a flag indicating whether the landmarks were succesfully extracted from the dataset.
 * @note DataHandler::readBarcodes needs to be called before this function since this function relies on the barcodes extracted.
 */
bool DataHandler::readLandmarks(const std::string& dataset) {

	std::string filename = dataset + "/Landmark_Groundtruth.dat";
	std::ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "[ERROR] Unable to open Landmarks file:" << filename << std::endl;
		return false;
	}
	/* Iterate through file line by line.*/
	int i = 0; 
	std::string line;

	while (std::getline(file, line)) {
		/* Ignore file comments. */
		if ('#' == line[0]) {
			continue;
		}

		/* Remove whitespaces */
		line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
		
		if (i >= TOTAL_LANDMARKS) { 
			std::cerr << "[ERROR] total read landmarks exceeds TOTAL_LANDMARKS\n";
			return false;
		}

		/* Set the landmark's ID */
		std::size_t start_index = 0; 
		std::size_t end_index = line.find('\t', 0);
		landmarks_[i].id = std::stoi(line.substr(start_index, end_index));

		/* Ensure that the barcodes have been extracted and set */
		if (barcodes_[landmarks_[i].id - 1] == 0) { 
			std::cerr << "[ERROR] Barcodes not correctly set" << std::endl;
			return false;
		}

		/* Set landmark's barcode */
		landmarks_[i].barcode = barcodes_[landmarks_[i].id - 1] ;
		
		/* Landmark x-coordinate [m] */
		start_index = end_index + 1; 
		end_index = line.find('\t', start_index);
		landmarks_[i].x = std::stod(line.substr(start_index, end_index - start_index));

		/* Landmark y-coordinate [m] */
		start_index = end_index + 1; 
		end_index = line.find('\t', start_index);
		landmarks_[i].y = std::stod(line.substr(start_index, end_index - start_index));

		/* Landmark x standard deviation [m] */
		start_index = end_index + 1; 
		end_index = line.find('\t', start_index);
		landmarks_[i].x_std_dev = std::stod(line.substr(start_index, end_index - start_index));

		/* Landmark y standard deviation [m] */
		start_index = end_index + 1; 
		end_index = line.find('\t', start_index);
		landmarks_[i++].y_std_dev = std::stod(line.substr(start_index, end_index - start_index));
	}

	file.close();

	return true;
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Groundtruth.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement will be assigned to.
 * @return a flag indicating whether the groundtruth was succesfully extracted from the dataset.
 * @details The data extracted form the Robotx_Groundtruth.dat contains the timestamp [s], x coordinate [m], y coordinate [m], and orientation [rad] of the robot x. These are used to populate the Robot::raw states member for a given robot in DataHandler::robots_.
 */
bool DataHandler::readGroundTruth(const std::string& dataset, int robot_id) {
	/* Clear all previous elements in the ground truth vector. */
	robots_[robot_id].raw.states.clear();

	/* Setup file for data extraction */
	std::string filename = dataset + "/Robot" + std::to_string(robot_id + 1) + "_Groundtruth.dat"; 
	std::ifstream file(filename);

	/* Check if the file could be opened */
	if (!file.is_open()) {
		std::cerr << "[ERROR] Unable to open Groundtruth file:" << filename << std::endl;
		return false;
	}

	/* Loop through each line in the file. */
	std::string line;
	while (std::getline(file, line)) {
		/* Ignore Comments */
		if ('#' == line[0]) {
			continue;
		}

		/* Remove whitespaces */
		line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

		/* Extract Data into thier respective variables */
		/* - Time */
		std::size_t start_index = 0;
		std::size_t end_index = line.find('\t', 0);
		double time = std::stod(line.substr(start_index, end_index));

		/* - x-coordinate [m] */
		start_index = end_index + 1;
		end_index = line.find('\t', start_index);
		double x_coordinate = std::stod(line.substr(start_index, end_index - start_index));

		/* - y-coordinate [m] */
		start_index = end_index + 1; 
		end_index = line.find('\t', start_index);
		double y_coordinate = std::stod(line.substr(start_index, end_index - start_index));

		/* - Orientaiton [rad] */
		start_index = end_index + 1;
		end_index = line.find('\t', start_index);
		double orientation = std::stod(line.substr(start_index, end_index - start_index));

		/* Populate robot states with exracted values. */
		robots_[robot_id].raw.states.push_back(Robot::State(time, x_coordinate, y_coordinate, orientation));
	}

	file.close();
	return true;
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Odometry.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement will be assigned to.
 * @return a flag indicating whether the robot odometry was succesfully extracted from the dataset.
 * @details The data extracted form the Robotx_Odometry.dat contains the timestamp [s], Forward Velocity [m/s], and Angular velocity [rad/s] of the measured odometry input into robot x. These are used to populate the Robot::raw odometry member for a given robot in DataHandler::robots_.
 */
bool DataHandler::readOdometry(const std::string& dataset, int robot_id) {
	/* Clear all previous elements in the odometry vector. */
	robots_[robot_id].raw.odometry.clear();

	/* Setup file for data extraction */
	std::string filename = dataset + "/Robot" + std::to_string(robot_id + 1) +"_Odometry.dat";
	std::fstream file(filename); 

	/* Check if the file could be opened */
	if (!file.is_open()) {
		std::cerr << "[ERROR] Unable to open Odometry file:" << filename << std::endl;
		return false;
	}

	/* Loop through each line in the file. */
	std::string line;
	while (std::getline(file, line)) {
		/* Ignore Comments */
		if ('#' == line[0]) {
			continue;
		}
		/* Remove Whitespace */
		line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

		/* Extract Data into thier respective variables */
		/* - Time */
		std::size_t start_index = 0;
		std::size_t end_index = line.find('\t', 0); 
		double time = std::stod(line.substr(start_index, end_index));

		/* - Forward Velocity [m/s] */
		start_index = end_index + 1;
		end_index = line.find('\t', start_index);
		double forward_velocity = std::stod(line.substr(start_index, end_index - start_index));
;
		/* - Angular Velocity [rad/s] */
		start_index = end_index + 1;
		end_index = line.find('\t', start_index);
		double angular_velocity = std::stod(line.substr(start_index, end_index - start_index));
;
		/* Populate the robot class with the extracted values. */
		robots_[robot_id].raw.odometry.push_back(Robot::Odometry(time, forward_velocity, angular_velocity));
	}

	file.close();
	return true;
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Measurement.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement will be assigned to.
 * @return a flag indicating whether the robot's measurements was succesfully extracted from the dataset.
 * @note The data values are tab seperated '\t'.
 * @note Grouping of measurements with the same time stamps does not occur during the reading. Therfore, the each member vector of measurements (subjects, ranges and bearings) are filled with only one value. The grouping by time stamp occurs in the DataHandler::syncData function.
 */
bool DataHandler::readMeasurements(const std::string& dataset, int robot_id) {
	/* Clear all previous elements in the measurement vector. */
	robots_[robot_id].raw.measurements.clear();
	robots_[robot_id].synced.measurements.clear();

	/* Setup file for data extraction */
	std::string filename = dataset + "/Robot" + std::to_string(robot_id+1) + "_Measurement.dat";
	std::fstream file(filename);

	/* Check if the file could be opened */
	if (!file.is_open()) {
		std::cerr << "[ERROR] Unable to open Measurement file:" << filename << std::endl;
		return false;
	}

	/* Loop through each line in the file. */
	std::string line;
	while (std::getline(file, line)) {
		/* Ignore Comments */
		if ('#' == line[0]) {
			continue;
		}
		/* Remove Whitespaces */
		line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

		/* Extract Data into thier respective variables.  */
		/* - Time [s]*/
		std::size_t start_index = 0;
		std::size_t end_index = line.find('\t', 0);
		double time = std::stod(line.substr(start_index, end_index));
		
		/* - Subject (ID) */
		start_index = end_index;
		end_index = line.find('\t', ++end_index);
		int subject = std::stoi(line.substr(start_index, end_index));

		/* If the subjects barcode extracted does not correspond to any of the barcodes extracted, then don't add the measurement. 
		 * NOTE: As far as I know, this occurs twice for robot 3 in dataset 1. It references barcode 43, which does not exists. */
		if (-1 == getID(subject)) {
			continue;
		}
		/* - Range [m] */
		start_index = end_index;
		end_index = line.find('\t', ++end_index);
		double range = std::stod(line.substr(start_index, end_index));

		/* - Bearing [rad] */
		start_index = end_index;
		end_index = line.find('\t', ++end_index);
		double bearing = std::stod(line.substr(start_index, end_index));

		robots_[robot_id].raw.measurements.push_back(Robot::Measurement(time, subject, range, bearing));
	}
	
	file.close();
	return true;
}

/**
 * @brief Syncs the time steps for the extracted data according to the specified sampling period.
 * @param[in] sample_period the desired sample period for resampling the data to sync the timesteps between the vehicles.
 * @note Synced values for the ground truth are saved in the DataHandler::robots_->groundtruth struct vector whereas synced odometry are saved in the DataHandler::robots_->synced struct
 */
void DataHandler::syncData(const double& sample_period) {
	/* Find the minimum and maximimum times in the datasets */
	double minimum_time = robots_[0].raw.states.front().time;
	double maximum_time = robots_[0].raw.states.back().time;

	for (int i = 1; i < TOTAL_ROBOTS; i++) {
		double robot_minimum_time = std::min({robots_[i].raw.states.front().time, robots_[i].raw.odometry.front().time, robots_[i].raw.measurements.front().time}); 
		double robot_maximum_time = std::min({robots_[i].raw.states.back().time, robots_[i].raw.odometry.back().time, robots_[i].raw.measurements.back().time}); 

		if (robot_minimum_time < minimum_time) {
			minimum_time = robot_minimum_time;
		}
		if (robot_maximum_time > maximum_time) {
			maximum_time = robot_maximum_time; 
		}
	}

	/* Subtract the minimum time from all timesteps to make t=0 the intial time of the system. */
	for  (int i = 0; i < TOTAL_ROBOTS; i++) {
		/* Set the loop length to the size of the largest vector */
		std::size_t dataset_size = std::max({robots_[i].raw.states.size(), robots_[i].raw.odometry.size(), robots_[i].raw.measurements.size()});

		for (std::size_t j = 0; j < dataset_size; j++) {
			if (j < robots_[i].raw.states.size()) {
				robots_[i].raw.states[j].time -= minimum_time; 
			}
			if (j < robots_[i].raw.odometry.size()) {
				robots_[i].raw.odometry[j].time -= minimum_time; 
			}
			if (j < robots_[i].raw.measurements.size()) {
				robots_[i].raw.measurements[j].time -= minimum_time; 
			}
		}
	}
	
	maximum_time -= minimum_time;

	/* Linear Interpolation. This section performs linear interpolation on the ground truth and odometry values to ensure that all robots have syncronised time steps. */
	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		/* Clear all previously interpolated values */
		robots_[id].groundtruth.states.clear();
		robots_[id].synced.odometry.clear();
		robots_[id].synced.measurements.clear();

		auto groundtruth_iterator = robots_[id].raw.states.begin();
		auto odometry_iterator = robots_[id].raw.odometry.begin();

		for (double t = 0.0; t <= maximum_time; t+=sample_period) {

			/* Find the first element that is larger than the current time step */
			groundtruth_iterator = std::find_if(groundtruth_iterator, robots_[id].raw.states.end(), [t](const Robot::State& element) {
			    return element.time > t;
			});
			/* If the element is the first item in the raw values, copy the raw values (no interpolation). This is assuming that the robot was stationary before its ground truth was recorded. */
			if (groundtruth_iterator == robots_[id].raw.states.begin()) {
				robots_[id].groundtruth.states.push_back( Robot::State(
					t, 
					robots_[id].raw.states.front().x, 
					robots_[id].raw.states.front().y, 
					robots_[id].raw.states.front().orientation
				)); 
				continue;
			}

			/* If the element is the larst item in the raw values, copy the raw values (no interpolation). This is assuming that the robot remains stationary after the ground truth recording ended. */
			else if (groundtruth_iterator == robots_[id].raw.states.end()) {
				robots_[id].groundtruth.states.push_back( Robot::State(
					t, 
					robots_[id].raw.states.back().x,
					robots_[id].raw.states.back().y, 
					robots_[id].raw.states.back().orientation
				));
				continue;
			}

			/* Interpolate the Groundtruth values */
			double interpolation_factor = (t -  (groundtruth_iterator-1)->time) / (groundtruth_iterator->time - (groundtruth_iterator -1)->time);

			double next_orientation = groundtruth_iterator->orientation;
			if (next_orientation - (groundtruth_iterator - 1)->orientation > 5) {
				next_orientation -= 2.0 * M_PI;
			}
			else if (next_orientation - (groundtruth_iterator - 1)->orientation < -5) {
				next_orientation += 2.0 * M_PI;
			}

			next_orientation = interpolation_factor * (next_orientation - (groundtruth_iterator - 1)->orientation) + (groundtruth_iterator - 1)->orientation;
			
			/* Normalise the orientation between PI and -PI (180 and -180 degrees respectively) */
			while (next_orientation >= M_PI) next_orientation -= 2.0 * M_PI;
			while (next_orientation < -M_PI) next_orientation += 2.0 * M_PI;

			robots_[id].groundtruth.states.push_back( Robot::State(
				t,
				interpolation_factor * (groundtruth_iterator->x - (groundtruth_iterator - 1)->x) + (groundtruth_iterator - 1)->x,
				interpolation_factor * (groundtruth_iterator->y - (groundtruth_iterator - 1)->y) + (groundtruth_iterator - 1)->y,
				next_orientation
			));

			/* The same process as above is repeated for the odometry, except assume the robot is stationary prior to ground truth readings */
			odometry_iterator = std::find_if(odometry_iterator, robots_[id].raw.odometry.end(), [t](const Robot::Odometry& element) {
			    return element.time > t;
			});

			if (odometry_iterator == robots_[id].raw.odometry.begin() || odometry_iterator == robots_[id].raw.odometry.end()-1) {
				robots_[id].synced.odometry.push_back(Robot::Odometry(t, 0, 0));
				continue;
			}

			/* Calculating Odometry Interpolation */
			interpolation_factor = (t -  (odometry_iterator-1)->time) / (odometry_iterator->time - (odometry_iterator -1)->time); 

			robots_[id].synced.odometry.push_back( Robot::Odometry(
				t,
				interpolation_factor * (odometry_iterator->forward_velocity - (odometry_iterator - 1)->forward_velocity) + (odometry_iterator - 1)->forward_velocity,
				interpolation_factor * (odometry_iterator->angular_velocity - (odometry_iterator - 1)->angular_velocity) + (odometry_iterator - 1)->angular_velocity
			));
		}

		/* The orginal UTIAS data extractor did NOT perform any linear interpolation on the meaurement values. The only action that was performed on the measurements was time stamp realignment according to the new timestamps. */
		robots_[id].synced.measurements.push_back( Robot::Measurement(
			std::floor(robots_[id].raw.measurements[0].time / sample_period + 0.5) * sample_period,
			robots_[id].raw.measurements[0].subjects,
			robots_[id].raw.measurements[0].ranges,
			robots_[id].raw.measurements[0].bearings
		));

		std::vector<Robot::Measurement>::iterator iterator = robots_[id].synced.measurements.end() - 1;

		/* Time stamp grouping: measurements with the same timestamps are grouped together to improve accessability. */
		for (std::size_t j = 1; j < robots_[id].raw.measurements.size(); j++) {
			double synced_time = std::floor(robots_[id].raw.measurements[j].time / sample_period + 0.5) * sample_period;
			/* If the current measurment has the same time stamp the previous measurment, join them. */
			if (synced_time == iterator->time) {
				iterator->subjects.push_back(robots_[id].raw.measurements[j].subjects[0]);
				iterator->ranges.push_back(robots_[id].raw.measurements[j].ranges[0]);
				iterator->bearings.push_back(robots_[id].raw.measurements[j].bearings[0]);
			}
			else {
				robots_[id].synced.measurements.push_back( Robot::Measurement(
					std::floor(robots_[id].raw.measurements[j].time / sample_period + 0.5) * sample_period,
					robots_[id].raw.measurements[j].subjects,
					robots_[id].raw.measurements[j].ranges,
					robots_[id].raw.measurements[j].bearings
				));
				iterator = robots_[id].synced.measurements.end() - 1;
			}
		}
	}
}

/**
 * @brief Utilises the extracted robots groundtruth position and heading values to calculate their associated groundtruth odometry values. 
 * @details The following expression is utilsed to calculate the groundtruth odomotery values using the groundtruth states values extracted from the dataset:
 * $$\begin{bmatrix} \omega_k & v_k \end{bmatrix}^\top = \begin{bmatrix} \text{arctan2}(\sin(\theta_{k+1} - \theta_{k}), \cos(\theta_{k+1} - \theta_{k})) / \Delta t & \sqrt{(x_{k+1} - x_k)^2 + (y_{k+1} - y_k)^2} \end{bmatrix}^\top, $$ 
 * where \f$k\f$ denotes the current time step; \f$\theta\f$ denotes the robot's orientation; \f$ y\f$ denotes the robot's y-coordinate; \f$\Delta t\f$ is the user defined sample period; \f$\omega\f$ and \f$v\f$ denotes the angular velocity and forward velocity of the robot respectively.
 */
void DataHandler::calculateGroundtruthOdometry() {
	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		robots_[id].groundtruth.odometry.clear();

		for (std::size_t k = 0; k < robots_[id].groundtruth.states.size() - 1; k++) {

			double x_difference = (robots_[id].groundtruth.states[k+1].x - robots_[id].groundtruth.states[k].x);
			double y_difference = (robots_[id].groundtruth.states[k+1].y - robots_[id].groundtruth.states[k].y);

			robots_[id].groundtruth.odometry.push_back( Robot::Odometry(
				robots_[id].groundtruth.states[k].time, 
				std::sqrt(x_difference*x_difference + y_difference*y_difference) / this->sampling_period_ ,
				std::atan2(std::sin(robots_[id].groundtruth.states[k+1].orientation - robots_[id].groundtruth.states[k].orientation), std::cos(robots_[id].groundtruth.states[k+1].orientation - robots_[id].groundtruth.states[k].orientation)) / this->sampling_period_ 
			));
		}
		/* NOTE: Since the last groundtruth odometry value can not be calculated, it is set equal to the synced measured value */
		robots_[id].groundtruth.odometry.push_back( Robot::Odometry(
			robots_[id].synced.odometry.back().time,
			robots_[id].synced.odometry.back().forward_velocity,
			robots_[id].synced.odometry.back().angular_velocity
		));
	}
}

/**
 * @brief Calculates the ground truth measurements for a given robot.
 * @details The following expression is utilised to calculate the groundtruth measurement values using the groundtruth robot state values extracted from the dataset:
 * $$\begin{bmatrix} r_{ij}^{(k)} & \phi_{ij}{(k)} \end{bmatrix}^\top = \begin{bmatrix} \sqrt{\left(x_i^{(k)} - x_j^{(k)}\right)^2  + \left(y_i^{(k)} - y_j^{(j)}\right)^2} & \text{atan2}\left(y_j^{(k)} - y_i^{(k)}, x_j^{(k)} - x_i^{(k)}\right) - \theta_i^{(k)}\end{bmatrix}^\top, $$
 * where \f$i\f$ denotes the ego robot; \f$j\f$ denotes the measured robot; \f$k\f$ denotes the current time step; \f$\theta\f$ denotes the robot's orientation; and \f$ y\f$ denotes the robot's y-coordinate.
 */
void DataHandler::calculateGroundtruthMeasurement() {
	for (int id = 0; id < TOTAL_ROBOTS; id++) {

		robots_[id].groundtruth.measurements.clear();
		auto iterator = robots_[id].groundtruth.measurements.begin();

		for (std::size_t k = 0; k < robots_[id].synced.measurements.size(); k++) {
			/* For loop iterator. Since the extracted data values ordered by time in ascending order, once a time value is found, prior time values do not need to be checked for newer time stamps. */
			size_t t = 0;
			/* Loop through each of the subjects and in the measurements and extract the landmarks */
			for (std::size_t s = 0; s < robots_[id].synced.measurements[k].subjects.size(); s++) {
				/* Get the subjects ID from its barcode. */
				int subject_ID =  getID(robots_[id].synced.measurements[k].subjects[s]);

				/* Find the value of the ground truth with the same time stamp as the measurement */
				for (; t < robots_[id].groundtruth.states.size(); t++) {
					if (std::round((robots_[id].groundtruth.states[t].time - robots_[id].synced.measurements[k].time) * 1000.0)/1000.0 == 0.0) {
						break;
					}
				}

				double x_difference;
				double y_difference;

				/* All robots have ID's [1,5] */
				if (subject_ID < 6) {
					subject_ID--;
					x_difference = robots_[subject_ID].groundtruth.states[t].x - robots_[id].groundtruth.states[t].x;
					y_difference = robots_[subject_ID].groundtruth.states[t].y - robots_[id].groundtruth.states[t].y; 
				}
				/* All landmarks have ID's [6,20] */
				else {
					subject_ID -= 6;
					x_difference = landmarks_[subject_ID].x - robots_[id].groundtruth.states[t].x;
					y_difference = landmarks_[subject_ID].y - robots_[id].groundtruth.states[t].y; 
				}

				double orientation = std::atan2(y_difference, x_difference) - robots_[id].groundtruth.states[t].orientation;
				while (orientation >= M_PI) orientation -= 2.0 * M_PI;
				while (orientation < -M_PI) orientation += 2.0 * M_PI;
				/*  */
				if (0 == s) {
					/* Create a new instance of the Measurement struct on the first */
					robots_[id].groundtruth.measurements.push_back( Robot::Measurement(
						robots_[id].synced.measurements[k].time,
						robots_[id].synced.measurements[k].subjects[s],
						std::sqrt(x_difference * x_difference + y_difference * y_difference),
						orientation
					)); 

					/* Move the iterator to the newly created instance*/
					iterator = robots_[id].groundtruth.measurements.end() - 1;
					continue;
				}

				iterator->subjects.push_back(robots_[id].synced.measurements[k].subjects[s]);
				iterator->ranges.push_back(std::sqrt(x_difference * x_difference + y_difference * y_difference));
				iterator->bearings.push_back(orientation);
			}
		}
	}
}

/**
 * @brief Calculates the relative distance of robots from an ego robot and saves the data. 
 * @details The relative distance between robots and the ego robot is calculated using the groundtruth state values extracted from the dataset for each robot.
 * @note This is only for robot 1 at this stage.
 */
void DataHandler::relativeRobotDistance() {
	std::ofstream robot_file;
	std::string filename = output_directory_ + "Relative_robot.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		return;
	}

	robot_file << "# Time [s]	Robot	Ranges [m]	Robot ID\n";
	for (std::size_t k =0; k < robots_[0].groundtruth.states.size(); k++) {
		for (int id = 0; id < TOTAL_ROBOTS; id++) {
			double x = robots_[0].groundtruth.states[k].x - robots_[id].groundtruth.states[k].x;
			double y = robots_[0].groundtruth.states[k].y - robots_[id].groundtruth.states[k].y;
			double range = std::sqrt(x*x + y*y);
			robot_file << robots_[0].groundtruth.states[k].time << '\t' << id << '\t' << range << '\t' << 1 << '\n';
		}
	}
	robot_file.close();
}

/**
 * @brief Calculates the relative distance of the landmarks from an ego robot and saves the data.
 * @details The relative distance between the landmarks and the ego robot is calculated using the landmarks and groundtruth state values extracted from the dataset for each landmark and the ego robot respectively.
 * @note This is only for robot 1 at this stage.
 */
void DataHandler::relativeLandmarkDistance() {
	std::ofstream robot_file;
	std::string filename = output_directory_ + "Relative_landmark.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		return;
	}

	robot_file << "# Time [s]	Landmark	Ranges [m]	Robot ID\n";
	for (std::size_t k =0; k < robots_[0].groundtruth.states.size(); k++) {
		for (int l = 0; l < TOTAL_LANDMARKS; l++) {
			double x = robots_[0].groundtruth.states[k].x - landmarks_[l].x;
			double y = robots_[0].groundtruth.states[k].y - landmarks_[l].y;
			double range = std::sqrt(x*x + y*y);
			robot_file << robots_[0].groundtruth.states[k].time << '\t' << l + 6 << '\t' << range << '\t' << 1 << '\n';
		}
	}
	robot_file.close();
}

/**
 * @brief Saves all the extracted and processed data in the DataHandler class after data extraction and processing.
 * @param[in] flag indicates whether the saving of all data was succesfull.
 */
void DataHandler::saveData(bool& flag) {

	output_directory_ = dataset_ + "/output/";
	std::filesystem::create_directories(output_directory_);

	saveStateData(flag);
	saveOdometryData(flag);
	saveMeasurementData(flag);

	saveErrorData(flag);

	double bin_size = 0.001;
	saveOdometryErrorPDF(flag, bin_size);
	saveMeasurementErrorPDF(flag, bin_size);

	saveRobotErrorStatistics();
	relativeLandmarkDistance();
	relativeRobotDistance();
}

/**
 * @brief Writes the synced (performed by DataHandler::syncData) and raw groundtruth robot state data extracted from the dataset after, which includes its x-coordinate, y-coordinate and heading.
 * @param[in] flag indicates whether saving the groundtruth state data was succesfull.
 */
void DataHandler::saveStateData(bool& flag) {

	std::ofstream robot_file;
	std::string filename = output_directory_ +  "Groundtruth-State.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	robot_file << "# Time [s]	x [m]	y [m]	orientation [rad]	Raw (r) / Synced (s)	Robot ID\n";
	/* Loop through the data structures for each robot */
	for (int id = 0; id < TOTAL_ROBOTS; id++ ) {
		/* Determine which dataset is larger and set that as the loop iterations */
		std::size_t largest_vector_size = std::max({robots_[id].raw.states.size(), robots_[id].groundtruth.states.size()});
		for (std::size_t k = 0; k < largest_vector_size; k++) {
			/* Write the raw ground truth file for the current timestep 'r'  */
			if (k < robots_[id].raw.states.size()) {
				robot_file << robots_[id].raw.states[k].time << '\t' << robots_[id].raw.states[k].x << '\t' << robots_[id].raw.states[k].y << '\t' << robots_[id].raw.states[k].orientation << '\t' << 'r' << '\t' << id + 1<< "\n";
			}
			
			/* Write the synced ground truth file for the current timestep 'r'  */
			if (k < robots_[id].groundtruth.states.size()){
				robot_file << robots_[id].groundtruth.states[k].time << '\t' << robots_[id].groundtruth.states[k].x << '\t' << robots_[id].groundtruth.states[k].y << '\t'<< robots_[id].groundtruth.states[k].orientation << '\t' << 's' << '\t' << id + 1 << '\n';
			}
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();

}

/**
 * @brief Saves the extracted measurement and calculated groundtruth measurement data from the DataHandler class into .dat files to be plotted by gnuplot.
 * @details Saves both the measurement data (as extracted from the dataset) and the calculated groundtruth measurement values (calculated by DataHandler::calculateGroundtruthMeasurement) into Measurement.dat and Groundtruth-Measurement.dat respectively. 
 * @param[in] flag indicates whether the saving of measurement data was succesfull.
 */
void DataHandler::saveMeasurementData(bool& flag) {

	std::ofstream robot_file;
	std::string filename = output_directory_ + "Measurement.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}
	robot_file << "# Time [s]	Subjects	Ranges [m]	Bearings [m]	Robot ID\n";
	/* Save the values of the raw and synced measurment values of a given robot into the same file with the last row indicating 'g' for raw  and 'i' for synced.*/
	for (int id = 0; id < TOTAL_ROBOTS; id++) {

		/* NOTE: when the "raw" measurement data structure is populated, it only adds one element to the members for each time stamp. After interpolation, these values are combined if they have the same time stamp.*/
		for (std::size_t k = 0; k < robots_[id].raw.measurements.size(); k++) {
			robot_file << robots_[id].raw.measurements[k].time << '\t' << robots_[id].raw.measurements[k].subjects[0] << '\t' << robots_[id].raw.measurements[k].ranges[0] << '\t' <<  robots_[id].raw.measurements[k].bearings[0] << '\t' << 'r' << '\t' << id + 1 << '\n';
		}
		for (std::size_t k = 0; k < robots_[id].synced.measurements.size(); k++) {
			for (std::size_t s = 0; s < robots_[id].synced.measurements[s].subjects.size(); s++) {
				robot_file << robots_[id].synced.measurements[s].time << '\t' << robots_[id].synced.measurements[s].subjects[s] << '\t' << robots_[id].synced.measurements[s].ranges[s] << '\t' << robots_[id].synced.measurements[s].bearings[s] << '\t' << 's' << '\t' << id + 1 << '\n';
			}
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();

	filename = output_directory_ + "Groundtruth-Measurement.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR] Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	/* Write the header. */
	robot_file << "# Time [s]	Subject	Landmark (l) / Robot (r)	Range [m]	Bearing [rad]	Robot ID\n";

	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		for (std::size_t k = 0; k < robots_[id].groundtruth.measurements.size(); k++ ) {
			for (std::size_t s = 0; s < robots_[id].groundtruth.measurements[k].subjects.size(); s++) {
				int subject_ID = getID(robots_[id].groundtruth.measurements[k].subjects[s]);
				if (subject_ID < 6) {
					robot_file << robots_[id].groundtruth.measurements[k].time << '\t' << robots_[id].groundtruth.measurements[k].subjects[s] << '\t' << 'r' << '\t' << robots_[id].groundtruth.measurements[k].ranges[s] << '\t' << robots_[id].groundtruth.measurements[k].bearings[s] << '\t' << id << '\n';
				}
				else {
					robot_file << robots_[id].groundtruth.measurements[k].time << '\t' << robots_[id].groundtruth.measurements[k].subjects[s] << '\t' << 'l' << '\t' << robots_[id].groundtruth.measurements[k].ranges[s] << '\t' << robots_[id].groundtruth.measurements[k].bearings[s] << '\t' << id << '\n';
				}
			}
		}

		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();
	
}

/**
 * @brief Saves the measured odometry data from the DataHandler class into a .dat file to be plotted by gnuplot.
 * @details Saves the odometry data (as extracted from the dataset) into Odometry.dat. 
 * @param[in] flag indicates whether saving the odometry data was succesfull.
 */
void DataHandler::saveOdometryData(bool& flag) {

	std::ofstream robot_file;
	std::string filename = output_directory_ + "Odometry.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	/* Write the header file. */
	robot_file << "# Time [s]	Forward Velocity [m/s]	Angular Velocity [rad/s]	Raw (r) / Synced (s)	Robot ID\n";

	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		std::size_t largest_vector_size = std::max({robots_[id].raw.odometry.size(), robots_[id].synced.odometry.size()});

		for (std::size_t k = 0; k < largest_vector_size; k++) {
			if (k < robots_[id].raw.odometry.size()) {
				robot_file << robots_[id].raw.odometry[k].time << '\t' << robots_[id].raw.odometry[k].forward_velocity << '\t' << robots_[id].raw.odometry[k].angular_velocity << '\t' << 'r' << '\t' << id + 1 << '\n';
			}
			
			if (k < robots_[id].synced.odometry.size()){
				robot_file << robots_[id].synced.odometry[k].time << '\t' << robots_[id].synced.odometry[k].forward_velocity << '\t' << robots_[id].synced.odometry[k].angular_velocity << '\t' << 's' << '\t' << id + 1 << '\n';
			}
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();
}

/**
 * @brief Saves calculated error between the measured data (as extracted form the dataset) and the calculated groundtruth values.
 * @details The error between the measured odometry and the calculated groundtruth odometry produced by Robot::calculateOdometryError and the error between the measured measurements and the groundtruth measurements produced bye Robot::calculateMeasurementError is saved into their respective .dat files.
 * @param[in] flag indicates whether saving the odometry and measurement error data was succesfull.
 */
void DataHandler::saveErrorData(bool& flag) {

	std::ofstream robot_file;
	std::string filename = output_directory_ + "Odometry-Error.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	/* Write file header. */
	robot_file << "# Time [s]	Forward Velocity [m/s]	Angular Velocity [rad/s]	Robot ID\n";

	/* Save the error values of the odometry.*/
	for (int id = 0; id < TOTAL_ROBOTS ; id ++) {
		for (std::size_t k = 0; k < robots_[id].error.odometry.size(); k++) {
			robot_file << robots_[id].error.odometry[k].time << '\t' << robots_[id].error.odometry[k].forward_velocity << '\t' << robots_[id].error.odometry[k].angular_velocity << '\t' << id + 1 << '\n';
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();

	filename = output_directory_ + "Measurement-Error.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	robot_file << "# Time [s]	Subject	Range [m]	Bearing[rad]	Robot ID\n";
	/* Save the error values of the odometry.*/
	for (int id = 0; id < TOTAL_ROBOTS ; id ++) {

		for (std::size_t k = 0; k < robots_[id].error.measurements.size(); k++) {
			for (std::size_t s = 0; s < robots_[id].error.measurements[k].subjects.size(); s++){
				robot_file << robots_[id].error.measurements[k].time << '\t' << robots_[id].error.measurements[k].subjects[s] << '\t' << robots_[id].error.measurements[k].ranges[s] << '\t' << robots_[id].error.measurements[k].bearings[s] << '\t' << id + 1 << '\n';
			} 
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();
}

/**
 * @brief Performs binning on the odometry error for the determination of a discretized Probability Density Function (PDF).
 * @param[in] flag indicates whether saving the odometry error PDF was succesfull.
 * @param[in] bin_size the size of the bins (denoting the range of values) that odometry measurement values gets grouped into. 
 * @note The bin count is actually the area contribution of the odometry error for a given odometry measurement. This means that the output is a discretized pdf, where the sum of the area of all the bins should equal 1. This is done for better visualisation when fitting a Gaussian curve to the data. 
 */
void DataHandler::saveOdometryErrorPDF(bool& flag, double bin_size) {
	std::string filename = output_directory_ + "Forward-Velocity-Error-PDF.dat";

	std::ofstream robot_file;
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	/* Write the header. */
	robot_file << "# Bin Centre	Bin Width	Bin Count	Robot ID\n";
	
	/* Save the plot data for the Forward Velocity Error  */
	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		std::unordered_map<int, double> forward_velocity_bin_counts;

		for (auto odometry: robots_[id].error.odometry) {
			int bin_index = static_cast<int>(std::floor(odometry.forward_velocity / bin_size));
			/* NOTE: The bin count is actually the area contribution of the odometry error for the given measurement. This means that the output is a discretized pdf, where the sum of the area of all the bins should equal 1. This is done for better visualisation when fitting a Gaussian curve to the data. */
			forward_velocity_bin_counts[bin_index] += 1.0/(robots_[id].error.odometry.size() * bin_size);
		}

		for (const auto& [bin_index, count] : forward_velocity_bin_counts) {
			double bin_start = bin_index * bin_size;
			double bin_end = bin_start + bin_size;

			robot_file << (bin_start + bin_end)/2 << '\t' << bin_size << "\t" << count << '\t' << id + 1 << '\n';
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}

	robot_file.close();

	filename = output_directory_ + "Angular-Velocity-Error-PDF.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	robot_file << "# Bin Centre	Bin Width	Count	Robot ID\n";

	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		/* Save the plot data for the Angular Velocity Error  */

		std::unordered_map<int, double> angular_velocity_bin_counts;

		for (auto odometry: robots_[id].error.odometry) {
			int bin_index = static_cast<int>(std::floor(odometry.angular_velocity / bin_size));
			angular_velocity_bin_counts[bin_index] += 1.0/(robots_[id].error.odometry.size() * bin_size);
		}

		for (const auto& [bin_index, count] : angular_velocity_bin_counts) {
			double bin_start = bin_index * bin_size;
			double bin_end = bin_start + bin_size;

			robot_file << (bin_start + bin_end)/2 << '\t' << bin_size << "\t" << count << '\t' << id + 1 << '\n';
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();
}

/**
 * @brief Performs binning on the measurement error for the determination of a discretized Probability Density Function (PDF).
 * @param[in] flag indicates whether saving the measurment error PDF was succesfull.
 * @param[in] bin_size the size of the bins (denoting the range of values) that measurement values gets grouped into. 
 * @note The bin count is actually the area contribution of the odometry error for a given odometry measurement. This means that the output is a discretized pdf, where the sum of the area of all the bins should equal 1. This is done for better visualisation when fitting a Gaussian curve to the data. 
 */
void DataHandler::saveMeasurementErrorPDF(bool& flag, double bin_size) {
	std::string filename = output_directory_ + "Range-Error-PDF.dat";

	std::ofstream robot_file;
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	robot_file << "# Bin Centre	Bin Width	Bin Count	Robot ID\n";
	/* Save the plot data for the Forward Velocity Error  */
	for (int id = 0; id < TOTAL_ROBOTS; id++) {

		double number_of_measurements = 0.0;
		for (std::size_t k = 0; k < robots_[id].error.measurements.size(); k++) {
			number_of_measurements += robots_[id].error.measurements[k].ranges.size();
		}

		std::unordered_map<int, double> range_bin_counts;
		for (auto measurement: robots_[id].error.measurements) {
			for (auto range :measurement.ranges) {
				int bin_index = static_cast<int>(std::floor(range / bin_size));
				range_bin_counts[bin_index] += 1.0/(number_of_measurements * bin_size);
			}
		}

		for (const auto& [bin_index, count] : range_bin_counts) {
			double bin_start = bin_index * bin_size;
			double bin_end = bin_start + bin_size;

			robot_file << (bin_start + bin_end)/2 << '\t' << bin_size << "\t" << count << '\t' << id + 1 << '\n';
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}

	robot_file.close();

	filename = output_directory_ + "Bearing-Error-PDF.dat";
	robot_file.open(filename);

	if (!robot_file.is_open()) {
		std::cerr << "[ERROR]: Could not create file: " << filename << std::endl;
		flag = false;
		return;
	}

	robot_file << "# Bin Centre	Bin Width	Count	Robot ID\n";

	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		/* Save the plot data for the Angular Velocity Error  */

		double number_of_measurements = 0.0;
		for (std::size_t k = 0; k < robots_[id].error.measurements.size(); k++) {
			number_of_measurements += robots_[id].error.measurements[k].ranges.size();
		}

		std::unordered_map<int, double> bearing_bin_counts;

		for (auto measurement: robots_[id].error.measurements) {
			for (auto bearing : measurement.bearings) {
				int bin_index = static_cast<int>(std::floor(bearing / bin_size));
				bearing_bin_counts[bin_index] += 1.0/(robots_[id].error.odometry.size() * bin_size);
			}
		}

		for (const auto& [bin_index, count] : bearing_bin_counts) {
			double bin_start = bin_index * bin_size;
			double bin_end = bin_start + bin_size;

			robot_file << (bin_start + bin_end)/2 << '\t' << bin_size << "\t" << count << '\t' << id + 1 << '\n';
		}
		/* Add two empty lines after robot entires for gnuplot */
		robot_file << '\n';
		robot_file << '\n';
	}
	robot_file.close();
}

/**
 * @brief Saves the sample mean and sample variance of the measured odometry and tracking data for each robot.
 */
void DataHandler::saveRobotErrorStatistics() {
	std::string filename = this->output_directory_ + "/Robot-Error-Statistics.dat";
	std::ofstream file(filename);

	if (!file.is_open()) {
		throw std::runtime_error("[ERROR] Unable to create file:  " + filename);
	}

	/* Write file header. */
	file << "# Robot ID	Forward Velocity Mean [m]	Forward Velocity Variance [m^2]	Angular Velocity Mean [rad]	Angular Veolcity [rad^2]	Range Mean [m]	Range Variance [m^2]	Bearing Mean [rad]	Bearing Variance [rad^2]\n";

	for (unsigned short int id = 0; id < TOTAL_ROBOTS; id++) {
		file << id << '\t' << robots_[id].forward_velocity_error.mean << '\t' << robots_[id].forward_velocity_error.variance << '\t' << robots_[id].angular_velocity_error.mean << '\t' << robots_[id].angular_velocity_error.variance << '\t' << robots_[id].range_error.mean << '\t' << robots_[id].range_error.variance << '\t' << robots_[id].bearing_error.mean << '\t' << robots_[id].bearing_error.variance << '\n';
		
		/* Two blank line for gnuplot to be able to automatically seperate data from different robots */
		file << '\n';
		file << '\n';
	}
	
}
/**
 * @brief Getter for the array of Barcodes.
 * @return a reference the barcodes integer vector extracted from the barcodes data file: Barcodes.dat.
 * @note if the dataset has not been set, the function will throw a std::runtime_error.
 */
std::vector<int>& DataHandler::getBarcodes() {
	if ("" ==  this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data."); 
	}
	return barcodes_;
}

/**
 * @brief Searches trough the list of barcodes to find the index ID of the robot or landmark.
 * @param[in] barcode the barcode value for which the ID needs to be found.
 * @return the ID of the robot of landmark. If the ID is not found -1 is returned.
 * @note the ID is one larger than it's index. Therefore, robot 4 has ID 4 and index 3 in the array DataHandler::robots_.
 * @note if the dataset has not been set, the function will throw a std::runtime_error.
 */
int DataHandler::getID(int barcode) {
	for (int i = 0; i < TOTAL_BARCODES; i++) {
		if (barcodes_[i] == barcode) {
			return (i + 1);
		}
	}
	return -1;
}

/**
 * @brief Getter for the array of Landmarks.
 * @return a reference to the Landmarks class vector, populated by extracting data form Landmarks.dat.
 */
std::vector<Landmark>& DataHandler::getLandmarks() {
	if ("" ==  this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data.");
	}
	return landmarks_;
}

/**
 * @brief Getter for the array of robots.
 * @return a reference to the Robot class vector, populated by extracting datefrom Robotx_Groundtruth.dat, Robotx_Odometry.dat, and Robotx_Measurement.dat.
 * @note if the dataset has not been set, the function will throw a std::runtime_error.
 */
std::vector<Robot>& DataHandler::getRobots() {
	if ("" == this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data.");
	}

	return robots_;
}

/**
 * @brief Getter for the DataHandler::sampling_period_ field.
 * @return the sampling period set by the user.
 * @note DataExtractor::sampling_period_ has a default value of 0.02.
 */
double DataHandler::getSamplePeriod() {
	return sampling_period_;
}

/**
 * @brief Getter for the DataHandler::TOTAL_ROBOTS field.
 * @return the number of robots set by the user dataset.
 * @note the field is initialised to zero, therefore if it is not set, a std::runtime_error will be throw.
 */
unsigned short int DataHandler::getNumberOfRobots() {
	if (0 == TOTAL_ROBOTS) {
		throw std::runtime_error("The total number of robots have not been set.");
	} 
	return TOTAL_ROBOTS;
}

/**
 * @brief Getter for the DataHandler::TOTAL_LANDMARKS field.
 * @return the number of landmarks set by the user or the dataset.
 * @note the field is initialised to zero, therefore if it is not set, a std::runtime_error will be throw.
 */
unsigned short int DataHandler::getNumberOfLandmarks() {
	if (0 == TOTAL_LANDMARKS) {
		throw std::runtime_error("The total number of landmarks have not been set.");
	} 
	return TOTAL_LANDMARKS;
}

/**
 * @brief Getter for the DataHandler::BARCODES field.
 * @return the number of landmarks set by the user or the dataset.
 * @note the field is initialised to zero, therefore if it is not set, a std::runtime_error will be throw.
 */
unsigned short int DataHandler::getNumberOfBarcodes() {
	if (0 == TOTAL_BARCODES) {
		throw std::runtime_error("The total number of barcodes have not been set.");
	} 
	return TOTAL_BARCODES;
}

