#include <../include/robot.h>
#include <stdexcept>
#include <string>
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


