/**
 * @file Robot.cpp
 * @brief Class implementation file responsible for the robot datastructor.
 * @details Houses the functionality for calculating the eror values and error
 * statistics for a given robot.
 * @author Daniel Ingham
 * @date 2025-04-23
 */
#include "Robot.h"
#include <algorithm> // std::sort
#include <iterator>  // std::iterator
#include <numeric>   // std::accumulate
#include <stdexcept> // std::runtime_error
#include <string>    // std::string
/**
 * @brief Default constructor.
 */
Robot::Robot() : id(0), barcode(0) {}

/**
 * @brief Default destructor.
 */
Robot::~Robot() {}

/**
 * @brief Calculates the absolute error between the groundtruth measurements
 * input odometry and measurement values.
 * @note If the function encounters an error, a std::runtime_error is thrown.
 */
void Robot::calculateSensorErrror() {
  calculateOdometryError();
  calculateMeasurementError();
  removeOutliers();
}

/**
 * @brief Calculates the difference between the calculated groundtruth and the
 * measured odometry value.
 */
void Robot::calculateOdometryError() {
  /* Check if the groundtruth has been set. */
  if (this->groundtruth.odometry.size() == 0) {
    throw std::runtime_error("Groundtruth odometry values for robot " +
                             std::to_string(this->id) + " have not been set.");
  }

  /* Check if the synced data has been set. */
  if (this->synced.odometry.size() == 0) {
    throw std::runtime_error("Synced odometry values for robot " +
                             std::to_string(this->id) + " have not been set.");
  }

  /* If the odometry error vector is not empty, empty it before calculation. */
  if (this->error.odometry.size() > 0) {
    this->error.odometry.clear();
  }

  this->error.odometry.reserve(this->groundtruth.odometry.size());

  /* Calculate odometry error for each measurement. */
  for (std::size_t k = 0; k < this->groundtruth.odometry.size() - 1; k++) {

    double forward_velocity = this->groundtruth.odometry[k].forward_velocity -
                              this->synced.odometry[k].forward_velocity;

    double angular_velocity = this->groundtruth.odometry[k].angular_velocity -
                              this->synced.odometry[k].angular_velocity;
    /* Normalise the error values between -pi and pi radians (-180 and 180
     * degrees respectively). */
    while (angular_velocity >= M_PI)
      angular_velocity -= 2.0 * M_PI;
    while (angular_velocity < -M_PI)
      angular_velocity += 2.0 * M_PI;

    this->error.odometry.push_back(Odometry(this->groundtruth.odometry[k].time,
                                            forward_velocity,
                                            angular_velocity));
  }
}

/**
 * @brief Calculates the difference between the measured range and bearing and
 * the calculated groundtruth.
 */
void Robot::calculateMeasurementError() {
  /* Check if the groundtruth has been set. */
  if (this->groundtruth.measurements.size() == 0) {
    throw std::runtime_error("Groundtruth measurement values for robot " +
                             std::to_string(this->id) + " have not been set.");
  }

  /* Check if the synced data has been set. */
  if (this->synced.measurements.size() == 0) {
    throw std::runtime_error("Synced measurement values for robot " +
                             std::to_string(this->id) + " have not been set.");
  }

  /* If the measurement error vector is not empty, empty it before calculation.
   */
  if (this->error.measurements.size() > 0) {
    this->error.measurements.clear();
  }

  /* Reserve memory for faster vector population. */
  this->error.measurements.reserve(this->groundtruth.measurements.size());

  /* Calculate Range and Bearing error for each measurement. */
  auto iterator = this->error.measurements.begin();
  for (std::size_t k = 0; k < this->groundtruth.measurements.size(); k++) {

    /* Loop through the subjects */
    bool first_item = true;
    for (std::size_t s = 0;
         s < this->groundtruth.measurements[k].subjects.size(); s++) {

      /* Check that the subjects match between the groundtruth and the synced
       * measurements.*/
      if (this->groundtruth.measurements[k].subjects[s] !=
          this->synced.measurements[k].subjects[s]) {
        throw std::runtime_error("The groundtruth subject barcode did not "
                                 "match the syned subject barcode.");
      }

      /* Ignore invalid measurements. These invalid measurements are explicitly
       * set by DataHandler::calculateGroundtruthMeasurement when an invalid
       * subject barcode is detected. */
      if (this->groundtruth.measurements[k].ranges[s] == -1.0 &&
          this->groundtruth.measurements[k].bearings[s] == 2 * M_PI) {
        continue;
      }

      /* If the measurement is the first for the time stamp, push back a new
       * instance of the measurment error. */
      if (first_item) {
        first_item = false;
        this->error.measurements.push_back(
            Measurement(this->groundtruth.measurements[k].time,
                        this->groundtruth.measurements[k].subjects[s],
                        this->groundtruth.measurements[k].ranges[s] -
                            this->synced.measurements[k].ranges[s],
                        this->groundtruth.measurements[k].bearings[s] -
                            this->synced.measurements[k].bearings[s]));
        iterator = this->error.measurements.end() - 1;
      } else {
        /* Otherwise append the measurement values to the existing measurments
           for the current time stamp. */
        iterator->subjects.push_back(
            this->groundtruth.measurements[k].subjects[s]);
        iterator->ranges.push_back(this->groundtruth.measurements[k].ranges[s] -
                                   this->synced.measurements[k].ranges[s]);
        iterator->bearings.push_back(
            this->groundtruth.measurements[k].bearings[s] -
            this->synced.measurements[k].bearings[s]);
      }
    }
  }
}

/**
 * @brief calculates the sample mean and sample variance of the error for all
 * the odometry and tracking measurements.
 * @details The sample mean and sample variance are calculate as
 * \f[\begin{align} \bar{x} &= \frac{\sum_{i\in n} x_i}{n}\\ \sigma^2 &=
 * \frac{\sum_{i\in n} (x_i - \bar{x})^2}{n-1},\end{align}\f] where \f$x_i\f$
 * denotes the \f$i\f$-th element in the sample of size \f$n\f$. The sample
 * variance formulation uses Bessel correction.
 * @note The calculation on the sample mean relies on the population of the
 * Robot::error vector and therefore, Robot::calculateMeasurementError needs to
 * be called before this function.
 */
void Robot::calculateSampleErrorStats() {

  /* Check if the eror measurement vector has been populated. */
  if (0 == this->error.odometry.size() ||
      0 == this->error.measurements.size()) {
    throw std::runtime_error(
        "Sensor Error has not been set: call "
        "Robot::calculateMeasurementError() before this funciton.");
  }

  /* Calculate forward velocity mean error. */
  double total_forward_velocity_error =
      std::accumulate(this->error.odometry.begin(), this->error.odometry.end(),
                      0.0, [](double acc, const Odometry &element) {
                        return acc + element.forward_velocity;
                      });

  this->forward_velocity_error.mean =
      total_forward_velocity_error / this->error.odometry.size();

  /* Forward velocity measurement error variance */
  double total_forward_velocity_deviation = std::accumulate(
      this->error.odometry.begin(), this->error.odometry.end(), 0.0,
      [&](double acc, const Odometry &element) {
        return acc + std::pow(element.forward_velocity -
                                  this->forward_velocity_error.mean,
                              2);
      });

  this->forward_velocity_error.variance =
      total_forward_velocity_deviation / (this->error.odometry.size() - 1);

  /* Calculate angular velocity mean error. */
  double total_angular_velocity_error =
      std::accumulate(this->error.odometry.begin(), this->error.odometry.end(),
                      0.0, [](double acc, const Odometry &element) {
                        return acc + element.angular_velocity;
                      });

  this->angular_velocity_error.mean =
      total_angular_velocity_error / this->error.odometry.size();

  /* Angular velocity measurement error variance */
  double total_angular_velocity_deviation = std::accumulate(
      this->error.odometry.begin(), this->error.odometry.end(), 0.0,
      [&](double acc, const Odometry &element) {
        return acc + std::pow(element.angular_velocity -
                                  this->angular_velocity_error.mean,
                              2);
      });

  this->angular_velocity_error.variance =
      total_angular_velocity_deviation / (this->error.odometry.size() - 1);

  /* Calculate range measurement mean error.
   * NOTE: The calculation of the total number of measurments is used for both
   * the range and bearing mean calculation using the assumption that the number
   * of range and bearings measurements are equal. This should always the case
   * as each range measurment will have a corresponding bearing. */
  double total_measurements = 0.0;
  double total_range_error = std::accumulate(
      this->error.measurements.begin(), this->error.measurements.end(), 0.0,
      [&](double acc, const Measurement &element) {
        total_measurements += element.ranges.size();
        return acc + std::accumulate(element.ranges.begin(),
                                     element.ranges.end(), 0.0);
      });

  this->range_error.mean = total_range_error / total_measurements;

  /* Range measurement error variance */
  double total_range_deviation = std::accumulate(
      this->error.measurements.begin(), this->error.measurements.end(), 0.0,
      [&](double acc, const Measurement &element) {
        return acc + std::pow(std::accumulate(element.ranges.begin(),
                                              element.ranges.end(), 0.0) -
                                  this->range_error.mean,
                              2);
      });
  this->range_error.variance = total_range_deviation / (total_measurements - 1);

  /* Calculate bearing measurement mean erorr. */
  double total_bearing_error = std::accumulate(
      this->error.measurements.begin(), this->error.measurements.end(), 0.0,
      [](double acc, const Measurement &element) {
        return acc + std::accumulate(element.bearings.begin(),
                                     element.bearings.end(), 0.0);
      });
  this->bearing_error.mean = total_bearing_error / total_measurements;

  /* Bearing measurement error variance */
  double total_bearing_deviation = std::accumulate(
      this->error.measurements.begin(), this->error.measurements.end(), 0.0,
      [&](double acc, const Measurement &element) {
        return acc + std::pow(std::accumulate(element.bearings.begin(),
                                              element.bearings.end(), 0.0) -
                                  this->bearing_error.mean,
                              2);
      });
  this->bearing_error.variance =
      total_bearing_deviation / (total_measurements - 1);
}

/**
 * @brief Calculates the median index for a given vector.
 * @param[in] lower The lower index of the vector.
 * @param[in] upper The upper index of the vector.
 * @return Index of the median.
 */
unsigned long int Robot::calculateMedian(const unsigned long int lower,
                                         const unsigned long int upper) {
  unsigned long int median = upper - lower + 1;
  median = (median + 1) / 2 - 1;
  return median + lower;
}

/**
 * @brief Calculates the median, first quartile, third quartile, and
 * inter-quartile range for a given sorted vector.
 * @param[in] sorted_vector A vector sorted in ascending order.
 * @param[out] error_statistics The struct of error statistics for a given
 * sensor, containing: the median, first quartile, third quartile, and
 * inter-quartile range of the sorted input vector.
 */
void Robot::calculateQuartiles(const std::vector<double> &sorted_vector,
                               Robot::ErrorStatistics &error_statistics) {

  unsigned long int index = calculateMedian(0, sorted_vector.size() - 1);

  error_statistics.median = sorted_vector[index];

  /* Calculate the first and third quartiles. */
  if (0 == sorted_vector.size() % 2) {
    error_statistics.q1 = sorted_vector.at(calculateMedian(0, index));
    error_statistics.q3 =
        sorted_vector.at(calculateMedian(index + 1, sorted_vector.size() - 1));
  } else {
    error_statistics.q1 = sorted_vector.at(calculateMedian(0, index - 1));
    error_statistics.q3 =
        sorted_vector.at(calculateMedian(index + 1, sorted_vector.size()));
  }
  /* Calculate the Inter-quartile range using quartile 1 and 3. */
  error_statistics.iqr = error_statistics.q3 - error_statistics.q1;
}

/**
 * @brief Sets the quartiles for the forward and angular velcoties as well as
 * the range and bearing.
 */
void Robot::setQuartiles() {
  /* Extract the data into seperate vectors to be sorted. */
  std::vector<double> forward_velocity;
  forward_velocity.reserve(this->error.odometry.size());

  std::vector<double> angular_velocity;
  angular_velocity.reserve(this->error.odometry.size());

  std::vector<double> range_errors;
  range_errors.reserve(this->raw.measurements.size());

  std::vector<double> bearing_errors;
  bearing_errors.reserve(this->raw.measurements.size());

  std::transform(this->error.odometry.begin(), this->error.odometry.end(),
                 std::back_inserter(forward_velocity),
                 [](const Robot::Odometry &odometry) {
                   return odometry.forward_velocity;
                 });

  std::transform(this->error.odometry.begin(), this->error.odometry.end(),
                 std::back_inserter(angular_velocity),
                 [](const Robot::Odometry &odometry) {
                   return odometry.angular_velocity;
                 });

  for (const auto &measurement_errors : this->error.measurements) {
    std::copy(measurement_errors.ranges.begin(),
              measurement_errors.ranges.end(),
              std::back_inserter(range_errors));
    std::copy(measurement_errors.bearings.begin(),
              measurement_errors.bearings.end(),
              std::back_inserter(bearing_errors));
  }

  /* Sort the vectors in ascending order. */
  std::sort(forward_velocity.begin(), forward_velocity.end());
  std::sort(angular_velocity.begin(), angular_velocity.end());

  std::sort(range_errors.begin(), range_errors.end());
  std::sort(bearing_errors.begin(), bearing_errors.end());

  /* Set the median, first quartile, third quartile, and inter-quartile range.
   */
  calculateQuartiles(forward_velocity, this->forward_velocity_error);
  calculateQuartiles(angular_velocity, this->angular_velocity_error);
  calculateQuartiles(range_errors, this->range_error);
  calculateQuartiles(bearing_errors, this->bearing_error);
}

/**
 * @brief Uses the interquartile range to remove outliers from the measurements.
 * @details This is done since some measurement errors are due to incorrect data
 * assocation (associaating the wrong barcode to a robot) and therefore give an
 * incorrect indication of the noise present in the range and bearing sensor.
 */
void Robot::removeOutliers() {
  setQuartiles();

  /* The Odometry Data does noth have significant outliers present for datasets
   * 1-8 */
  /* Remove Measurement Outliers */
  for (auto error_measurement_iterator = this->error.measurements.begin();
       error_measurement_iterator != this->error.measurements.end();) {

    /*  NOTE: The upper and lower bound for the range (10) and bearing (20) were
     * manually tuned. */
    double range_lower_bound =
        this->range_error.q1 - 10 * this->range_error.iqr;
    double range_upper_bound =
        this->range_error.q3 + 10 * this->range_error.iqr;

    double bearing_lower_bound =
        this->bearing_error.q1 - 20 * this->bearing_error.iqr;
    double bearing_upper_bound =
        this->bearing_error.q3 + 20 * this->bearing_error.iqr;

    auto subjects_iterator = error_measurement_iterator->subjects.begin();
    auto ranges_iterator = error_measurement_iterator->ranges.begin();
    auto bearings_iterator = error_measurement_iterator->bearings.begin();

    for (; subjects_iterator != error_measurement_iterator->subjects.end();) {

      if (*ranges_iterator < range_lower_bound ||
          *ranges_iterator > range_upper_bound ||
          *bearings_iterator < bearing_lower_bound ||
          *bearings_iterator > bearing_upper_bound) {

        subjects_iterator =
            error_measurement_iterator->subjects.erase(subjects_iterator);
        ranges_iterator =
            error_measurement_iterator->ranges.erase(ranges_iterator);
        bearings_iterator =
            error_measurement_iterator->bearings.erase(bearings_iterator);

        continue;
      }

      ++subjects_iterator;
      ++ranges_iterator;
      ++bearings_iterator;
    }

    /* If the measurement has no subjects left. Remove the timestep. */
    if (0U == error_measurement_iterator->subjects.size()) {
      error_measurement_iterator =
          this->error.measurements.erase(error_measurement_iterator);
    } else {
      ++error_measurement_iterator;
    }
  }
}
/**
 * @brief calculates the difference between the groundtruth and the synced
 * states.
 * @details The synced states are calculated by some localisation filter and not
 * the robot or the DataHandler.
 */
void Robot::calculateStateError() {
  /* Check if the synced have been set. */
  if (this->synced.states.empty()) {
    std::runtime_error("Synced states have to been set.");
  }

  /* Calculate the error between the groundtruth and the states. */
  for (unsigned long k = 0; k < this->synced.states.size(); k++) {
    double orientation_error = this->groundtruth.states[k].orientation -
                               this->synced.states[k].orientation;

    /* Normalise the orientation error between -180 and 180. */
    while (orientation_error >= M_PI)
      orientation_error -= 2.0 * M_PI;

    while (orientation_error < -M_PI)
      orientation_error += 2.0 * M_PI;

    this->error.states.push_back(
        State(this->groundtruth.states[k].time,
              this->groundtruth.states[k].x - this->synced.states[k].x,
              this->groundtruth.states[k].y - this->synced.states[k].y,
              orientation_error));
  }
}
