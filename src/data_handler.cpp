/**
 * @file data_extractor.cpp
 * @brief Class implementation file responsible for exracting the ground-truth, odometry and measurement data from the UTIAS multi-robot localisation dataset.
 * @details The class extracts the data from the groundtruth, measurement, and populates three main 
 * @author Daniel Ingham
 * @date 2025-03-28
 */

#include "../include/data_handler.h"

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
DataHandler::DataHandler(const std::string& dataset,const double& sample_period){
	setDataSet(dataset, sample_period);
}

/**
 * @brief Extracts data from the barcodes data file: Barcodes.dat.
 * @param[in] dataset path to the dataset folder.
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
			std::cerr << "[ERROR] total read barcodes exceeds TOTAL_BARCODES\n";
			return false;
		}

		/* Extract barcodes into barcodes array */
		barcodes_[i++] = std::stoi(line.substr(line.find('\t', 0))) ;
	}

	file.close();

	return true;
}

/**
 * @brief Extracts data from the landmarks data file: Landmark_Groundtruth.dat.
 * @param[in] dataset path to the dataset folder.
 * @note DataHandler::readBarcodes needs to be called before this function since this function relies on the barcodes it extracted.
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

		/* Populate robot groundtruth with exracted values. */
		robots_[robot_id].raw.states.push_back(Robot::State(time, x_coordinate, y_coordinate, orientation));
	}

	file.close();
	return true;
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Odometry.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement will be assigned to.
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
		/* Populate the robot struct with the extracted values. */
		robots_[robot_id].raw.odometry.push_back(Robot::Odometry(time, forward_velocity, angular_velocity));
	}

	file.close();
	return true;
}

/**
 * @brief Extracts data from the groundtruth data file: Robotx_Measurement.dat.
 * @param[in] dataset path to the dataset folder.
 * @param[in] robot_id the ID of the robot for which the extracted measurement will be assigned to.
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

		/* Extract Data into thier respective variables */
		/* - Time [s]*/
		std::size_t start_index = 0;
		std::size_t end_index = line.find('\t', 0);
		double time = std::stod(line.substr(start_index, end_index));
		
		/* - Subject (ID) */
		start_index = end_index;
		end_index = line.find('\t', ++end_index);
		int subject = std::stoi(line.substr(start_index, end_index));

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
 * @brief Extracts data from the all files in the specified dataset folder.
 * @param[in] dataset dataset path to the dataset folder.
 * @param[in] sample_period the desired sample period for resampling the data to sync the timesteps between the vehicles.
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
		throw std::runtime_error("Dataset file path does not exist"); 
	}

	/* Set the sample period for this dataset. */
	this->sampling_period_ = sample_period;
	
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
}

/**
 * @brief Getter for the array of Barcodes.
 * @return Returns an integer pointer to the array of barcodes extracted from the barcodes data file: Barcodes.dat.
 */
int* DataHandler::getBarcodes() {
	if ("" ==  this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data."); 
	}
	return barcodes_;
}

/**
 * @brief returns the ID of the robot or landmark based on the barcode provided. 
 * @note the ID is one larger than it's index. Therefore, robot 4 has ID 4 and index 3 in the array DataHandler::robots_.
 */
int DataHandler::getID(int barcode) {
	for (int i = 0; i < TOTAL_BARCODES; i++) {
		if (barcodes_[i] == barcode) {
			return i + 1;
		}
	}
	throw std::runtime_error("The barcode specified does not exist.");
}
/**
 * @brief Getter for the array of Landmarks.
 * @return Returns a pointer to the Landmarks structure member, populated by extracting data form Landmarks.dat.
 */
DataHandler::Landmark* DataHandler::getLandmarks() {
	if ("" ==  this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data.");
	}
	return landmarks_;
}

/**
 * @brief Getter for the array of robots.
 * @return Returns a pointer to the Robot structure member, populated by extracting datefrom Robotx_Groundtruth.dat, Robotx_Odometry.dat, and Robotx_Measurement.dat.
 */
Robot* DataHandler::getRobots() {
	if ("" == this->dataset_) {
		throw std::runtime_error("Dataset has not been specified during object instantiation. Please ensure you call void setDataSet(std::string) before attempting to get data.");
	}

	return robots_;
}
double DataHandler::getSamplePeriod() {
	return sampling_period_;
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
	for (int i = 0; i < TOTAL_ROBOTS; i++) {
		auto groundtruth_iterator = robots_[i].raw.states.begin();
		auto odometry_iterator = robots_[i].raw.odometry.begin();

		for (double t = 0.0; t <= maximum_time; t+=sample_period) {

			/* Find the first element that is larger than the current time step */
			groundtruth_iterator = std::find_if(groundtruth_iterator, robots_[i].raw.states.end(), [t](const Robot::State& element) {
			    return element.time > t;
			});
			/* If the element is the first item in the raw values, copy the raw values (no interpolation). This is assuming that the robot was stationary before its ground truth was recorded. */
			if (groundtruth_iterator == robots_[i].raw.states.begin()) {
				robots_[i].groundtruth.states.push_back( Robot::State(
					t, 
					robots_[i].raw.states.front().x, 
					robots_[i].raw.states.front().y, 
					robots_[i].raw.states.front().orientation
				)); 
				continue;
			}

			/* If the element is the larst item in the raw values, copy the raw values (no interpolation). This is assuming that the robot remains stationary after the ground truth recording ended. */
			else if (groundtruth_iterator == robots_[i].raw.states.end()) {
				robots_[i].groundtruth.states.push_back( Robot::State(
					t, 
					robots_[i].raw.states.back().x,
					robots_[i].raw.states.back().y, 
					robots_[i].raw.states.back().orientation
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

			robots_[i].groundtruth.states.push_back( Robot::State(
				t,
				interpolation_factor * (groundtruth_iterator->x - (groundtruth_iterator - 1)->x) + (groundtruth_iterator - 1)->x,
				interpolation_factor * (groundtruth_iterator->y - (groundtruth_iterator - 1)->y) + (groundtruth_iterator - 1)->y,
				next_orientation
			));

			/* The same process as above is repeated for the odometry, except assume the robot is stationary prior to ground truth readings */
			odometry_iterator = std::find_if(odometry_iterator, robots_[i].raw.odometry.end(), [t](const Robot::Odometry& element) {
			    return element.time > t;
			});

			if (odometry_iterator == robots_[i].raw.odometry.begin() || odometry_iterator == robots_[i].raw.odometry.end()-1) {
				robots_[i].synced.odometry.push_back(Robot::Odometry(t, 0, 0));
				continue;
			}

			/* Calculating Odometry Interpolation */
			interpolation_factor = (t -  (odometry_iterator-1)->time) / (odometry_iterator->time - (odometry_iterator -1)->time); 

			robots_[i].synced.odometry.push_back( Robot::Odometry(
				t,
				interpolation_factor * (odometry_iterator->forward_velocity - (odometry_iterator - 1)->forward_velocity) + (odometry_iterator - 1)->forward_velocity,
				interpolation_factor * (odometry_iterator->angular_velocity - (odometry_iterator - 1)->angular_velocity) + (odometry_iterator - 1)->angular_velocity
			));
		}

		/* The orginal UTIAS data extractor did NOT perform any linear interpolation on the meaurement values. The only action that was performed on the measurements was time stamp realignment according to the new timestamps. */
		robots_[i].synced.measurements.push_back( Robot::Measurement(
			std::floor(robots_[i].raw.measurements[0].time / sample_period + 0.5) * sample_period,
			robots_[i].raw.measurements[0].subjects,
			robots_[i].raw.measurements[0].ranges,
			robots_[i].raw.measurements[0].bearings
		));

		std::vector<Robot::Measurement>::iterator iterator = robots_[i].synced.measurements.end() - 1;

		/* Time stamp grouping: measurements with the same timestamps are grouped together to improve accessability. */
		for (std::size_t j = 1; j < robots_[i].raw.measurements.size(); j++) {
			double synced_time = std::floor(robots_[i].raw.measurements[j].time / sample_period + 0.5) * sample_period;
			if (synced_time == iterator->time) {
				iterator->subjects.push_back(robots_[i].raw.measurements[j].subjects[0]);
				iterator->ranges.push_back(robots_[i].raw.measurements[j].ranges[0]);
				iterator->bearings.push_back(robots_[i].raw.measurements[j].bearings[0]);
			}
			else {
				robots_[i].synced.measurements.push_back( Robot::Measurement(
					std::floor(robots_[i].raw.measurements[j].time / sample_period + 0.5) * sample_period,
					robots_[i].raw.measurements[j].subjects,
					robots_[i].raw.measurements[j].ranges,
					robots_[i].raw.measurements[j].bearings
				));
				iterator = robots_[i].synced.measurements.end() - 1;
			}
		}
	}
}

/**
 * @brief Utilises the extracted robots groundtruth position and heading values to calculate their associated groundtruth odometry values. 
 * @details The following expression is utilsed to calculate the odomotery values
 * $$\begin{bmatrix} \omega_k & v_k \end{bmatrix}^\top = \begin{bmatrix} \atan(\sin(\theta_{k+1} - \theta_{k}), \cos(\theta_{k+1} - \theta_{k})) / \Delta t & sqrt{(x_{k+1} - x_k)^2 + (y_{k+1} - y_k)^2} \end{bmatrix}^\top, $$ 
 * where \f$k\f$ denotes the current time step; \f$\theta\f$ denotes the robot's orientation; \f$ y\f$ denotes the robot's y-coordinate; \f$\Delta t\f$ is the user defined sample period; \f$\omega\f$ and \f$v\f$ denotes the angular velocity and forward velocity of the robot respectively.
 */
void DataHandler::calculateGroundtruthOdometry() {
	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		for (std::size_t k = 0; k < robots_[id].groundtruth.states.size() - 1; k++) {

			double x_difference = (robots_[id].groundtruth.states[k+1].x - robots_[id].groundtruth.states[k].x);
			double y_difference = (robots_[id].groundtruth.states[k+1].y - robots_[id].groundtruth.states[k].y);

			robots_[id].groundtruth.odometry.push_back( Robot::Odometry(
				robots_[id].groundtruth.states[k].time, 
				std::sqrt(x_difference*x_difference + y_difference*y_difference) / this->sampling_period_ ,
				std::atan2(std::sin(robots_[id].groundtruth.states[k+1].orientation - robots_[id].groundtruth.states[k].orientation), std::cos(robots_[id].groundtruth.states[k+1].orientation - robots_[id].groundtruth.states[k].orientation)) / this->sampling_period_ 

			));
		}
		/* NOTE: Since the last groundtruth value can not be calculated, it is set equal to the synced measured value */
		robots_[id].groundtruth.odometry.push_back( Robot::Odometry(
			robots_[id].synced.odometry.back().time,
			robots_[id].synced.odometry.back().forward_velocity,
			robots_[id].synced.odometry.back().angular_velocity
		));
	}
}


/**
 * @brief Calculates the ground truth measurements for a given robot. 
 */
void DataHandler::calculateGroundtruthMeasurement() {
	for (int id = 0; id < TOTAL_ROBOTS; id++) {
		auto iterator = robots_[id].groundtruth.measurements.begin();
		for (std::size_t k = 0; k < robots_[id].synced.measurements.size(); k++) {

			/* Loop through each of the subjects and in the measurements and extract the landmarks */
			for (std::size_t s = 0; s < robots_[id].synced.measurements[k].subjects.size(); s++) {
				int subject_ID =  getID(robots_[id].synced.measurements[k].subjects[s]);
				double x_difference;
				double y_difference;
				/* All robots have ID's [1,5] */
				if (id < 6) {
					subject_ID--;
					x_difference = robots_[id].groundtruth.states[k].x - robots_[subject_ID].groundtruth.states[k].x;
					y_difference = robots_[id].groundtruth.states[k].y - robots_[subject_ID].groundtruth.states[k].y; 
				}
				/* All landmarks have ID's [6,20] */
				else {
					subject_ID -= 6;
					x_difference = robots_[id].groundtruth.states[k].x - landmarks_[subject_ID].x;
					y_difference = robots_[id].groundtruth.states[k].y - landmarks_[subject_ID].y; 
				}
				/*  */
				if (s == 0) {
					/* Create a new instance of the Measurement struct on the first */
					robots_[id].groundtruth.measurements.push_back( Robot::Measurement(
						robots_[id].synced.measurements[k].time,
						robots_[id].synced.measurements[k].subjects[s],
						std::sqrt(x_difference * x_difference + y_difference * y_difference),
						std::atan2(y_difference, x_difference) - robots_[id].groundtruth.states[k].orientation
					)); 

					/* Move the iterator to the newly created instance*/
					iterator = robots_[id].groundtruth.measurements.end() - 1;
					continue;
				}

				iterator->subjects.push_back(robots_[id].synced.measurements[k].subjects[s]);
				iterator->ranges.push_back(std::sqrt(x_difference * x_difference + y_difference * y_difference));
				iterator->bearings.push_back(std::atan2(y_difference, x_difference) - robots_[id].groundtruth.states[k].orientation);

			}
		}
	} 
}


