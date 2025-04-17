#include <../include/robot.h>
#include <cmath>
#include <numeric>

/**
 * @brief Default constructor.
 */
Robot::Robot() {
}

/**
 * @brief Default destructor
 */
Robot::~Robot() {
}

/**
 * @brief Calculates the absolute error between the groundtruth measurements input odometry and measurement values.
 */
void Robot::calculateMeasurementError() {

	/* Check if the groundtruth has been set. */
	if (this->groundtruth.odometry.size() == 0) {
		throw std::runtime_error("Groundtruth state values for robot " + std::to_string(this->id) + " have not been set."); 
	}

	/* Check if the synced data has been set. */
	if (this->synced.odometry.size() == 0) {
		throw std::runtime_error("Synced values for robot " + std::to_string(this->id) + " have not been set."); 
	} 

	/* Check if the groundtruth has been set. */
	if (this->groundtruth.measurements.size() == 0) {
		throw std::runtime_error("Groundtruth state values for robot " + std::to_string(this->id) + " have not been set."); 
	}

	/* Check if the synced data has been set. */
	if (this->synced.measurements.size() == 0) {
		throw std::runtime_error("Synced values for robot " + std::to_string(this->id) + " have not been set."); 
	} 

	/* If the odometry error vector is not empty, empty it before calculation. */
	if (this->error.odometry.size() > 0) {
		this->error.odometry.clear();
	}

	/* Calculate odometry error for each measurement. */
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


	/* If the measurement error vector is not empty, empty it before calculation. */
	if (this->error.measurements.size() > 0) {
		this->error.measurements.clear();
	}

	/* Calculate measurement error for each measurement. */
	auto iterator = this->error.measurements.begin();
	for (std::size_t k = 0; k < this->groundtruth.measurements.size(); k++) {

		/* Loop through the subjects */
		for (std::size_t s = 0; s < this->groundtruth.measurements[k].subjects.size(); s++) {
			
			/* Update the error statistics. */
			this->range_error.mean += (this->groundtruth.measurements[k].ranges[s] - this->synced.measurements[k].ranges[s]);
			this->bearing_error.mean += (this->groundtruth.measurements[k].bearings[s] - this->synced.measurements[k].bearings[s]);

			/* If the measurement is the first for the time stamp, push back a new instance of the measurment error. */
			if (s == 0) {
				this->error.measurements.push_back( Measurement(
					this->groundtruth.measurements[k].time,
					this->groundtruth.measurements[k].subjects[s],
					this->groundtruth.measurements[k].ranges[s] - this->synced.measurements[k].ranges[s],
					this->groundtruth.measurements[k].bearings[s] - this->synced.measurements[k].bearings[s]
				));
				iterator = this->error.measurements.end() - 1;
			}
			/* Otherwise append the measurement values to the existing measurments for the current time stamp. */
			else {
				iterator->subjects.push_back(this->groundtruth.measurements[k].subjects[s]);
				iterator->ranges.push_back(this->groundtruth.measurements[k].ranges[s] - this->synced.measurements[k].ranges[s]);
				iterator->bearings.push_back(this->groundtruth.measurements[k].bearings[s] - this->synced.measurements[k].bearings[s]);
			}
		}
	}

	setErrorSampleMean();
	setErrorSampleVariance();
}

/**
 * @brief calculates the sample mean of the error for all the odometry and tracking measurements.
 * @details The sample mean is calculate as 
 * $$ \bar{x} = \frac{\sum_{i\in n} x_i}{n},$$ 
 * where \f$x_i\f$ denotes the $i$-th element in the sample of size \f$n\f$.
 * @note The calculation on the sample mean relies on the population of the Robot::error vector and therefore, Robot::calculateMeasurementError needs to be called before this function.
 */
void Robot::setErrorSampleMean() {

	double total_forward_velocity_error = std::accumulate(this->error.odometry.begin(), this->error.odometry.end(), 0.0, [](double acc, Odometry element) {
		return acc + element.forward_velocity;
	});
	this->forward_velocity_error.mean = total_forward_velocity_error / this->error.odometry.size();

	double total_angular_velocity_error = std::accumulate(this->error.odometry.begin(), this->error.odometry.end(), 0.0, [](double acc, Odometry element) {
		return acc + element.angular_velocity;
	});

	this->angular_velocity_error.mean = total_angular_velocity_error / this->error.odometry.size();

	/* NOTE: The calculation of the total number of measurments is used for both the range and bearing mean calculation using the assumption that the number of range and bearings measurements are equal. This should always the case as each range measurment will have a corresponding bearing. */
	double total_measurements = 0.0;
	double total_range_error = std::accumulate(this->error.measurements.begin(), this->error.measurements.end(), 0.0, [&](double acc, Measurement element) {
		total_measurements +=  element.ranges.size();
		return acc + std::accumulate(element.ranges.begin(), element.ranges.end(), 0.0);
	});

	this->range_error.mean = total_range_error / total_measurements;

	double total_bearing_error = std::accumulate(this->error.measurements.begin(), this->error.measurements.end(), 0.0, [](double acc, Measurement element) {
		return acc + std::accumulate(element.bearings.begin(), element.bearings.end(), 0.0);
	});

	this->bearing_error.mean = total_bearing_error / total_measurements;
}

/**
 * @brief Calculates the sample variance of the error for all the odometry and tracking measurements.
 * @details The variance is calculated as 
 * $$\sigma^2 = \frac{\sum_{i\in n} (x_i - \bar{x})^2}{n-1}, $$ 
 * where \f$x_i\f$ denotes the $i$-th element in the sample of size \f$n\f$. This function using the Bessel correction formulation for the calculation of the sample mean.
 * @note the sample variance calculation is reliant on the sample mean. Therefore, Robot::setErrorSampleMean needs to be called first before calling this function.
 */
void Robot::setErrorSampleVariance() {

	double total_forward_velocity_deviation = std::accumulate(this->error.odometry.begin(), this->error.odometry.end(), 0.0, [&](double acc, Odometry element) {
		return acc + std::pow(element.forward_velocity - this->forward_velocity_error.mean, 2) ;
	});
	this->forward_velocity_error.variance = total_forward_velocity_deviation / (this->error.odometry.size() - 1);

	double total_angular_velocity_deviation = std::accumulate(this->error.odometry.begin(), this->error.odometry.end(), 0.0, [&](double acc, Odometry element) {
		return acc + std::pow(element.angular_velocity - this->angular_velocity_error.mean, 2);
	});
	this->angular_velocity_error.variance = total_angular_velocity_deviation / (this->error.odometry.size() - 1);
	
	double total_measurements = 0.0;
	double total_range_deviation = std::accumulate(this->error.measurements.begin(), this->error.measurements.end(), 0.0, [&](double acc, Measurement element) {
		total_measurements += element.ranges.size();
		return acc + std::pow(std::accumulate(element.ranges.begin(), element.ranges.end(), 0.0) - this->range_error.mean, 2);
	});
	this->range_error.variance = total_range_deviation / (total_measurements - 1);

	double total_bearing_deviation = std::accumulate(this->error.measurements.begin(), this->error.measurements.end(), 0.0, [&](double acc, Measurement element) {
		return acc + std::pow(std::accumulate(element.bearings.begin(), element.bearings.end(), 0.0) - this->bearing_error.mean, 2);
	}); 

	this->bearing_error.variance = total_bearing_deviation / (total_measurements - 1);

}
