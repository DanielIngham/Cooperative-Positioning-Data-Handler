#include <../include/robot.h>
#include <iostream>

Robot::Robot() {
}

Robot::~Robot() {
}

/**
 * @brief Calculates the absolute error between the groundtruth odometry values and the values measured.
 */
void Robot::calculateOdometryError() {

	/* Check if the groundtruth has been set. */
	if (this->groundtruth.odometry.size() == 0) {
		throw std::runtime_error("Groundtruth state values for robot " + std::to_string(this->id) + " have not been set."); 
	}

	/* Check if the synced data has been set. */
	if (this->synced.odometry.size() == 0) {
		throw std::runtime_error("Synced values for robot " + std::to_string(this->id) + " have not been set."); 
	} 

	/* If the odometry error vector is not empty, empty it before calculation. */
	if (this->error.odometry.size() > 0) {
		this->error.odometry.clear();
	}

	/* Calculate groundtruth error. */
	for (std::size_t k = 0; k < this->groundtruth.odometry.size() - 1; k++) {
		/* Ignore stationary odometry values before the system starts and after it ends. These readings cause a disproportionate amount of zero error readings. */
		if (this->synced.odometry[k].angular_velocity != 0 && this->synced.odometry[k].forward_velocity != 0) {
			this->error.odometry.push_back( Odometry(
				this->groundtruth.odometry[k].time,
				this->groundtruth.odometry[k].forward_velocity - this->synced.odometry[k].forward_velocity,
				this->groundtruth.odometry[k].angular_velocity - this->synced.odometry[k].angular_velocity
			));
		}
	}
}


void Robot::calculateMeasurementError() {

	/* Check if the groundtruth has been set. */
	if (this->groundtruth.measurements.size() == 0) {
		throw std::runtime_error("Groundtruth state values for robot " + std::to_string(this->id) + " have not been set."); 
	}

	/* Check if the synced data has been set. */
	if (this->synced.measurements.size() == 0) {
		throw std::runtime_error("Synced values for robot " + std::to_string(this->id) + " have not been set."); 
	} 

	/* If the measurement error vector is not empty, empty it before calculation. */
	if (this->error.measurements.size() > 0) {
		this->error.measurements.clear();
	}

	/* Calculate groundtruth error. */
	auto iterator = this->error.measurements.begin();
	for (std::size_t k = 0; k < this->groundtruth.measurements.size() - 1; k++) {
		/* Loop through the subjects */
		for (std::size_t s = 0; s < this->groundtruth.measurements[k].subjects.size(); s++) {
			if (s == 0) {
				this->error.measurements.push_back( Measurement(
					this->groundtruth.measurements[k].time,
					this->groundtruth.measurements[k].subjects[s],
					this->groundtruth.measurements[k].ranges[s] - this->synced.measurements[k].ranges[s],
					this->groundtruth.measurements[k].bearings[s] - this->synced.measurements[k].bearings[s]
				));
				iterator = this->error.measurements.end() - 1;
				continue;
			}

			iterator->subjects.push_back(this->groundtruth.measurements[k].subjects[s]);
			iterator->ranges.push_back(this->groundtruth.measurements[k].ranges[s] - this->synced.measurements[k].ranges[s]);
			iterator->bearings.push_back(this->groundtruth.measurements[k].bearings[s] - this->synced.measurements[k].bearings[s]);
		}
	}
}
