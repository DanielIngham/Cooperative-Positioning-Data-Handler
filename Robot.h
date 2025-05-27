/**
 * @file Robot.h
 * @brief Header file of the Robot class.
 * @author Daniel Ingham
 * @date 2025-04-16
 */
#ifndef INCLUDE_INCLUDE_ROBOT_H_
#define INCLUDE_INCLUDE_ROBOT_H_

#include <cmath>  // std::atan2
#include <vector> // std::vector

/**
 * @class Robot
 * @brief Houses all data and functionality related to a given robot in a
 * multi-robot localisation environment
 */
class Robot {
public:
  Robot();
  Robot(Robot &&) = default;
  Robot(const Robot &) = default;
  Robot &operator=(Robot &&) = default;
  Robot &operator=(const Robot &) = default;
  ~Robot();

  /**
   * @brief Numerical identifier for the robot.
   */
  int id;

  /**
   * @brief  Barcode associated with the robot. This is what the other robots
   * will read during there operation to identify each other.
   */
  int barcode;

  /**
   * @brief Data attributes for a single groundtruth reading extracted from
   * Robotx_Groundtruth.dat.
   * @note According the UTIAS website, the groundtruth readings are accurate to
   * the order of 1mm (1E^-3 m). Additionally, the NTP deamon used to
   * synchronise the clocks between robots has an average timing error of 1ms
   * (1E-3 seconds).
   */
  struct State {
    /* The following attributes are extracted directly from the robots
     * groundtruth datafile. */
    double time; ///< Time stamp of the ground truth reading [s].

    double x;           ///< Robot Groundtruth x-coordinate [m].
    double y;           ///< Robot Groundtruth y-coordinate [m].
    double orientation; ///< Robot Groundtruth orientation [rad].

    /** @brief Default constructor */
    State();

    /**
     * @brief Constructor for convenient population of
     * DataExtractor::robots_.raw.ground_truth .
     */
    State(double time_, double x_, double y_, double orientation_)
        : time(time_), x(x_), y(y_), orientation(orientation_) {}
  };

  /**
   * @brief Data attributes of a single odometry reading extracted from
   * Robotx_Odometry.dat.
   */
  struct Odometry {
    double time;             ///< Time stamp of the odometry readings [s].
    double forward_velocity; ///< The robots forward velocity [m/s].
    double angular_velocity; ///< The robots angular velocity [rad/s].

    Odometry();
    /**
     * @brief Constructor for convenient population of
     * DataExtractor::robots_.raw.odometry .
     */
    Odometry(double time_, double forward_velocity_, double angular_velocity_)
        : time(time_), forward_velocity(forward_velocity_),
          angular_velocity(angular_velocity_) {};
  };

  /**
   * @brief Data attributes of the relative distance measurement taken at a
   * single timestep in Robotx_Measurements.
   * @note This structure combines multiple measurements with the same time
   * stamp. There it is assumed that the vectors: subjects, ranges, and bearings
   * are in the same order.
   */
  struct Measurement {
    double time; ///< Time stamp of the measurement [s].
    std::vector<unsigned short>
        subjects; ///< The Barcode of the other robots being measured.
    std::vector<double> ranges;   ///< The measured ranges to the subjects [m]
    std::vector<double> bearings; ///< The bearings from the subjects [rad]

    /**
     * @brief Constructor that allows for the copying of measurement structures.
     */
    Measurement(const Robot::Measurement &measurement)
        : time(measurement.time), subjects(measurement.subjects),
          ranges(measurement.ranges), bearings(measurement.bearings) {};

    /**
     * @brief Constructor that allows for the copying of measurement structure
     * elements.
     */
    Measurement(double time_, const std::vector<unsigned short> &subjects_,
                const std::vector<double> &ranges_,
                const std::vector<double> &bearings_)
        : time(time_), subjects(subjects_), ranges(ranges_),
          bearings(bearings_) {};
    /**
     * @brief Constructor for convenient population of raw measurments that have
     * only one subject.
     */
    Measurement(double time_, unsigned short subject_, double range_,
                double bearing_)
        : time(time_) {
      subjects.push_back(subject_);
      ranges.push_back(range_);
      bearings.push_back(bearing_);
    };
  };

  /**
   * @brief Vectors related to the robot data.
   */
  struct RobotData {
    std::vector<State>
        states; ///< All groundtruth values extracted for the given robot.
    std::vector<Odometry>
        odometry; ///< All odometry inputs extracted for the given robot.
    std::vector<Measurement>
        measurements; ///< All measurements taken by the given robot.
  };
  /** @brief The raw data extracted from the dataset's .dat files. */
  RobotData raw;
  /** @brief The odometry and measurement values with synced timesteps. */
  RobotData synced;
  /** The groundtruth */
  RobotData groundtruth;
  /** @brief The difference between the ground truth and the synced data */
  RobotData error;

  /**
   * @brief Error statistics used by filters for inference.
   * @details It is often assumed that all errors are caussed by white Gaussian
   * distributed noise, which forms the foundation for the devolopment of
   * Bayesian filtering. For this reason, the robots noise statistics are
   * calculated.
   */
  struct ErrorStatistics {
    double mean = 0.0;     ///< The sample mean of the error.
    double variance = 0.0; ///< The sample standard deviation of the error.

    double median = 0.0; ///< The sample median of the error.
    double q1 = 0.0;     ///< The first quartile.
    double q3 = 0.0;     ///< The third quartile.
    double iqr = 0.0;    ///< Inter Quartile Range.
  };

  /** @brief  Error associated with the range measurements. */
  ErrorStatistics range_error;
  /** @brief Error associated with the bearing measurements. */
  ErrorStatistics bearing_error;
  /** @brief Error associated with the forward velocity input. */
  ErrorStatistics forward_velocity_error;
  /** @brief Error associated with the angular velocity input. */
  ErrorStatistics angular_velocity_error;

  void calculateSensorErrror();
  void calculateSampleErrorStats();
  void calculateStateError();

private:
  unsigned long int calculateMedian(const unsigned long int,
                                    const unsigned long int);
  void calculateQuartiles(const std::vector<double> &, ErrorStatistics &);
  void setQuartiles();

  void calculateOdometryError();
  void calculateMeasurementError();

  void removeOutliers();
};

#endif // INCLUDE_INCLUDE_ROBOT_H_
