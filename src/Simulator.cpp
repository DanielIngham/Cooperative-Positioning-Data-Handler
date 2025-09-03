/**
 * The class simulates data in the same form as the UTIAS mulirobot localsiation
 * and mapping dataset.
 * @file Simulator.cpp
 * @brief Class implementation file responsible for simulating the data for
 * multi-robot localisation and mapping.
 * @author Daniel Ingham
 * @date 2025-04-25
 */

#include "Simulator.h"

#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>

namespace Data {

/**
 * @brief Default constructor.
 */
Simulator::Simulator() {}
/*
 * @brief Default destructor.
 */
Simulator::~Simulator() {}

/**
 * @brief Constructor which populates the data for the robots and landmarks.
 */
Simulator::Simulator(const unsigned long int data_points, double sample_period,
                     std::vector<Robot> &robots,
                     std::vector<Landmark> &landmarks,
                     std::vector<unsigned short int> &barcodes,
                     const unsigned long seed)
    : data_points_(data_points), sample_period_(sample_period),
      robots_(&robots), landmarks_(&landmarks), barcodes_(&barcodes) {

  setSimulation(data_points_, sample_period_, robots, landmarks, barcodes,
                seed);
}

/**
 * @brief Populates the robots and landmarks with simulated values.
 */
void Simulator::setSimulation(const unsigned long int data_points,
                              double sample_period, std::vector<Robot> &robots,
                              std::vector<Landmark> &landmarks,
                              std::vector<unsigned short int> &barcodes,
                              const unsigned long seed) {
  setGeneratorSeed(seed);

  data_points_ = data_points;
  sample_period_ = sample_period;

  total_landmarks = landmarks.size();
  total_robots = robots.size();
  total_barcodes_ = this->total_landmarks + this->total_robots;

  robots_ = &robots;
  landmarks_ = &landmarks;
  barcodes_ = &barcodes;

  assignVectorMemory();
  setBarcodes();
  setErrorStatistics();
  setLandmarkPositions();
  setRobotsInitalState();
  setRobotOdometryAndState();
  setRobotMeasurement();
  addGaussianNoise();
}

/**
 * @brief Assigns the memory sizes for the vectors to be populated by the
 * simulator.
 */
void Simulator::assignVectorMemory() {
  for (unsigned short id{}; id < total_robots; ++id) {
    (*robots_)[id].groundtruth.states.reserve(data_points_);
    (*robots_)[id].synced.odometry.reserve(data_points_);

    /* Set the first element to the origin. This will be overwritten with random
     * values in Simulator::setRobots(). */
    (*robots_)[id].groundtruth.states.emplace_back(Robot::State{
        .time = .0,
        .x = .0,
        .y = .0,
        .orientation = .0,
    });
  }
}

/**
 * @brief Sets the barcodes for each of the robots and landmarks.
 * @details The are first assigned to the robots and then the landmarks.The
 * barcodes set in the simulator are the same as the ID. The only reason the
 * barcodes are set at all is for compatability with the original UTIAS
 * multirobot localisation and mapping dataset.
 * @note The orginal dataset incorporated a checksum where the robot barcodes
 * add up to 5 and the landmark barcodes add up to 6 or 9. Since this is a
 * simulation, this checksum is not nessary and therefore not incorporated.
 */
void Simulator::setBarcodes() {
  for (unsigned short id{}; id < total_barcodes_; ++id) {

    (*barcodes_)[id] = id + 1;

    if (id < total_robots) {
      (*robots_)[id].barcode = (*barcodes_)[id];
    } else {
      (*landmarks_)[id - total_robots].barcode = (*barcodes_)[id];
    }
  }
}

/**
 * @brief Sets the robots error variances and landmarks position error
 * standard deviation.
 */
void Simulator::setErrorStatistics() {

  /* Set the landmarks standard deviation */
  std::uniform_real_distribution<double> deviation(
      std::sqrt(variance_.landmarks[MIN]), std::sqrt(variance_.landmarks[MAX]));

  /* Loop through each landmark and set the id and standard deviation */
  for (unsigned short i{}; i < total_landmarks; i++) {

    /* Set the ID for each Landmark */
    (*landmarks_)[i].id = total_robots + (i + 1);

    /* Set the variance for each landmark */
    (*landmarks_)[i].x_std_dev = deviation(generator_);
    (*landmarks_)[i].y_std_dev = deviation(generator_);
  }

  /* Set all the robot ID's and variance robots */
  std::uniform_real_distribution<double> forward_velocity_error(
      variance_.forward_velocity[MIN], variance_.forward_velocity[MAX]);

  std::uniform_real_distribution<double> angular_velocity_error(
      variance_.angular_velocity[MIN], variance_.angular_velocity[MAX]);

  std::uniform_real_distribution<double> range_error(variance_.range[MIN],
                                                     variance_.range[MAX]);

  std::uniform_real_distribution<double> bearing_error(variance_.bearing[MIN],
                                                       variance_.bearing[MAX]);

  for (unsigned short id{}; id < total_robots; id++) {
    (*robots_)[id].id = id + 1;

    (*robots_)[id].forward_velocity_error.variance =
        forward_velocity_error(generator_);

    (*robots_)[id].angular_velocity_error.variance =
        angular_velocity_error(generator_);

    (*robots_)[id].range_error.variance = range_error(generator_);
    (*robots_)[id].bearing_error.variance = bearing_error(generator_);
  }
}

/**
 * @brief Sets the x and y coordinate for the number of landmarks provided.
 */
void Simulator::setLandmarkPositions() {

  /* Generate random x, y positions within the simulation region and some
   * buffer: 0.5 metres.*/
  double position_padding{0.5};

  std::uniform_real_distribution<double> position_x(
      position_padding, limits_.width - position_padding);

  std::uniform_real_distribution<double> position_y(
      position_padding, limits_.height - position_padding);

  /* Set the first landmark with a random x,y coordinate pair. */
  (*landmarks_).front().x = position_x(generator_);
  (*landmarks_).front().y = position_y(generator_);

  /* Loop through each landmark and assign a x,y coordinate that is at least
   * 2m apart from all other landmarks. */

  auto timer_start = std::chrono::high_resolution_clock::now();

  for (unsigned short i{1U}; i < total_landmarks; i++) {
    auto current_time = std::chrono::high_resolution_clock::now();

    auto timer_duration = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - timer_start);

    if (timer_duration.count() > 1) {
      position_x.param(std::uniform_real_distribution<double>::param_type(
          .0, limits_.width));

      position_y.param(std::uniform_real_distribution<double>::param_type(
          .0, limits_.height));
    } else if (timer_duration.count() > 2) {
      std::cout << timer_duration.count() << std::endl;
      position_x.param(std::uniform_real_distribution<double>::param_type(
          -1.0, limits_.width + 1));

      position_y.param(std::uniform_real_distribution<double>::param_type(
          -1.0, this->limits_.height + 1));
    }

    /* Generate a random coordinate for the landmark*/
    (*landmarks_)[i].x = position_x(this->generator_);
    (*landmarks_)[i].y = position_y(this->generator_);

    for (unsigned short j{}; j < i; j++) {

      /* Check that the new point is far enough away from other points
       * randomly choosen. */
      double x_difference = (*landmarks_)[i].x - (*landmarks_)[j].x;
      double y_difference = (*landmarks_)[i].y - (*landmarks_)[j].y;
      double distance =
          std::sqrt(x_difference * x_difference + y_difference * y_difference);

      /* If the point generated is too close to other points, restart the
       * process. */
      double minimum_distance{2.0};

      if (distance < minimum_distance) {
        i--;
        break;
      }
    }
  }
}

/**
 * @brief Sets the intial unique x,y coordinate and orienation for the number
 * of robots provided.
 */
void Simulator::setRobotsInitalState() {

  /* Set up random function for x and y position to fall within 1 metre of the
   * simulation limits. */
  static const double position_padding{1.0};
  std::uniform_real_distribution<double> position_x(
      position_padding, limits_.width - position_padding);

  std::uniform_real_distribution<double> position_y(
      position_padding, limits_.height - position_padding);

  std::uniform_real_distribution<double> orientation(-M_PI, M_PI);

  /* Set the initial value for the robot 1 state. */
  bool unique;
  do {
    unique = true;

    /* Overwrite the origin values set in Robot::assignVectorMemory. */
    Robot::State &first_robot_state =
        (*robots_).front().groundtruth.states.front();

    first_robot_state = {
        .time = .0,
        .x = position_x(generator_),
        .y = position_y(generator_),
        .orientation = orientation(generator_),
    };

    for (const auto &landmark : (*landmarks_)) {
      /* Check that the new point is far enough away from other points
       * randomly choosen. */
      double x_difference{landmark.x - first_robot_state.x};
      double y_difference{landmark.y - first_robot_state.y};
      double distance{
          std::sqrt(x_difference * x_difference + y_difference * y_difference)};

      /* If the point generated is too close to other points, restart the
       * process. */
      if (distance < 2.0) {
        unique = false;
        break;
      }
    }
  } while (!unique);

  /* Set the initial values up for the remaining robots. */
  for (unsigned short id{1U}; id < total_robots; id++) {

    /* Overwrite the origin values set in Robot::assignVectorMemory. */
    unique = true;
    Robot::State &current_robot_pose =
        (*robots_)[id].groundtruth.states.front();

    current_robot_pose = Robot::State{
        .time = .0,
        .x = position_x(generator_),
        .y = position_y(generator_),
        .orientation = orientation(generator_),
    };

    /* Check that the position is far enough away from other robots */
    for (unsigned short j{}; j < id; j++) {

      const Robot::State &other_agent_pose =
          (*robots_)[j].groundtruth.states.front();

      /* Check that the new point is far enough away from other points
       * randomly choosen. */
      double x_difference{current_robot_pose.x - other_agent_pose.x};

      double y_difference{current_robot_pose.y - other_agent_pose.y};

      double distance =
          std::sqrt(x_difference * x_difference + y_difference * y_difference);

      /* If the point generated is too close to other points, restart the
       * process. */
      if (distance < 1.0) {
        id--;
        unique = false;
        break;
      }
    }

    /* If the robot position is too close to other robots, there is no need to
     * check the other landmarks since the position will need to be updated
     * anyway. */
    if (false == unique) {
      continue;
    }

    for (const auto &landmark : (*landmarks_)) {
      /* Check that the new point is far enough away from other points
       * randomly choosen. */
      double x_difference{landmark.x - current_robot_pose.x};
      double y_difference{landmark.y - current_robot_pose.y};
      double distance{
          std::sqrt(x_difference * x_difference + y_difference * y_difference)};

      /* If the point generated is too close to other points, restart the
       * process. */
      if (distance < 2.0) {
        id--;
        break;
      }
    }
  }
}

/**
 * @brief Sets the odometry values (forward and angular velocity) for the
 * number of robots provided along with the corresponding states.
 * @details The setting of the robot odometry and state were coupled to allow
 * for the odometry values to change depending on the robot state. If the
 * robot is too close to the edge of the simulation area, the odometry values
 * are changed to steer to robot back to the centre of the area.
 * @note Simulator::setRobotsInitalState needs to be called before this
 * function. If this is not done, a std::runtime_error will be thrown.
 */
void Simulator::setRobotOdometryAndState() {

  /* Create a centre point. */
  double centre_x{limits_.width / 2.0}, centre_y{limits_.height / 2.0};

  /* Set up random number generation functions.
   * The walk length denotes the number of samples for which an input is
   * applied. */
  const unsigned short minimum_walk{20U}, maximum_walk{500U};

  std::uniform_int_distribution<unsigned short> walk_length(minimum_walk,
                                                            maximum_walk);

  /* Random generator for for the robot's intial forward velocity. */
  const double &min_initial_forward_velocity{limits_.forward_velocity / 2.0},
      &max_initial_forward_velocity{limits_.forward_velocity};

  std::uniform_real_distribution<double> initial_forward_velocity(
      min_initial_forward_velocity, max_initial_forward_velocity);

  /* Random generator for the angular velocity input */
  const double &min_initial_angular_input{-limits_.angular_velocity},
      &max_initial_angular_input{limits_.angular_velocity};

  std::uniform_real_distribution<double> angular_velocity_input(
      min_initial_angular_input, max_initial_angular_input);

  /* Random generator for the forward velocity innput */
  std::uniform_real_distribution<double> forward_velocity_input(-0.05, 0.05);

  /* Loop through each robot and assign them odometry inputs. */
  for (auto &robot : (*robots_)) {

    /* Preallocate the space for the inference data that the filters/smoothers
     * will populate. */
    robot.synced.states.resize(data_points_);

    /* Check if the intial states for every robots has bee set. */
    if (robot.groundtruth.states.empty()) {
      throw std::runtime_error(
          "The initial state of Robot " + std::to_string(robot.id) +
          " was not set. Call Simulator::setRobotsInitalState before calling "
          "Simulator::setRobotOdometry");
    }

    /* Populate the robot's inital input. */
    robot.groundtruth.odometry.push_back(Robot::Odometry{
        .time = 0.0,
        .forward_velocity = initial_forward_velocity(generator_),
        .angular_velocity = 0.0,
    });

    /* Calculate the resulting state from this those intpus */

    double x_position{
        robot.groundtruth.states.front().x +
        robot.groundtruth.odometry.front().forward_velocity * sample_period_ *
            std::cos(robot.groundtruth.states.front().orientation)};

    double y_position{
        robot.groundtruth.states.front().y +
        robot.groundtruth.odometry.front().forward_velocity * sample_period_ *
            std::sin(robot.groundtruth.states.front().orientation)};

    double orientation{robot.groundtruth.states.front().orientation +
                       sample_period_ *
                           robot.groundtruth.odometry.front().angular_velocity};

    robot.groundtruth.states.emplace_back(
        Robot::State{.time = sample_period_,
                     .x = x_position,
                     .y = y_position,
                     .orientation = orientation});

    /* Assign a random walk length at random  */
    unsigned short random_walk_duration{walk_length(this->generator_)};

    /* Generate random odometry inputs for every datapoint. */
    double angular_input{};

    for (size_t k{1}; k < data_points_; ++k) {

      double forward_adjustment{0.0};

      /* If the robot is about to leave the simulation boundaries, the robot
       * should be guided back towards the centre of the simulation area. */
      static const double position_buffer{1.0};

      Robot::State &groundtruth_pose = robot.groundtruth.states.at(k);

      if (groundtruth_pose.x < position_buffer ||
          groundtruth_pose.x > (limits_.width - position_buffer) ||
          groundtruth_pose.y < position_buffer ||
          groundtruth_pose.y > (limits_.height - position_buffer)) {

        /* Calculate the distance from the centre points and get the angle
         * adjustment. */
        double x_difference = centre_x - groundtruth_pose.x;

        double y_difference = centre_y - groundtruth_pose.y;

        double bearing_for_centre = std::atan2(y_difference, x_difference) -
                                    groundtruth_pose.orientation;

        /* Normalise the orientation */
        while (bearing_for_centre >= M_PI)
          bearing_for_centre -= 2.0 * M_PI;
        while (bearing_for_centre <= -M_PI)
          bearing_for_centre += 2.0 * M_PI;

        /* If the the bearing from the centre point is less than approximately
         * 10 degrees positive or negative, there is no need to make further
         * adjustments. */
        /* Gradually correct the orientation through adjustments to the
         * angular velocity. */
        /* NOTE: the an adjustement to forward velocity is not made. */
        angular_input = bearing_for_centre / (M_PI / limits_.angular_velocity);

        /* Inputs are applied for a random number of samples. */
      } else if ((k % random_walk_duration) == 0) {

        /* Assign a new velocity adjustment */
        forward_adjustment = forward_velocity_input(generator_);
        angular_input = angular_velocity_input(generator_);

        /* Assign a new random walk length at random  */
        random_walk_duration = walk_length(generator_);
      }

      /* Boundary checks on new odometry values. */
      double new_forward_velocity{
          robot.groundtruth.odometry.at(k - 1).forward_velocity +
          forward_adjustment};

      double &new_angular_velocity = angular_input;

      /* NOTE: It is assumed that the robots cannot reverse. */
      if (new_forward_velocity > limits_.forward_velocity) {
        new_forward_velocity = limits_.forward_velocity;

      } else if (new_forward_velocity < 0.0) {
        new_forward_velocity = 0.0;
      }

      if (new_angular_velocity > limits_.angular_velocity) {
        new_angular_velocity = limits_.angular_velocity;

      } else if (new_angular_velocity < -limits_.angular_velocity) {
        new_angular_velocity = -limits_.angular_velocity;
      }

      /* Populate odometry with new values. */
      robot.groundtruth.odometry.emplace_back(Robot::Odometry{
          .time = sample_period_ * k,
          .forward_velocity = new_forward_velocity,
          .angular_velocity = new_angular_velocity,
      });

      /* Prevents the groundtruth from having one more value than the
       * odometry.
       */
      if (robot.groundtruth.states.size() == data_points_) {
        continue;
      }

      /* Calculate the resulting state from this those intpus */
      x_position = robot.groundtruth.states.at(k).x +
                   robot.groundtruth.odometry.at(k).forward_velocity *
                       sample_period_ *
                       std::cos(robot.groundtruth.states.at(k).orientation);

      y_position = robot.groundtruth.states.at(k).y +
                   robot.groundtruth.odometry.at(k).forward_velocity *
                       sample_period_ *
                       std::sin(robot.groundtruth.states.at(k).orientation);

      orientation =
          robot.groundtruth.states.at(k).orientation +
          sample_period_ * robot.groundtruth.odometry.at(k).angular_velocity;

      /* Normalise orienation between -180 and 180. */
      while (orientation >= M_PI)
        orientation -= 2.0 * M_PI;
      while (orientation < -M_PI)
        orientation += 2.0 * M_PI;

      robot.groundtruth.states.emplace_back(Robot::State{
          .time = sample_period_ * k,
          .x = x_position,
          .y = y_position,
          .orientation = orientation,
      });

      /* Set the time of the synced state data points. */
      robot.synced.states[k].time = sample_period_ * k;
    }
  }
}

/**
 * @brief Calculates the groundtruth range bearing of robots from one another
 * that fall within a given range.
 */
void Simulator::setRobotMeasurement() {

  const unsigned short &measurement_to_odometry_ratio =
      SimulationDefaults::kmeasurement_to_odometry_ratio;

  const double &max_range = SimulationDefaults::kmax_range;

  for (unsigned long k{}; k < data_points_; k++) {

    if ((k % measurement_to_odometry_ratio) != 0) {
      continue;
    }

    /* Determine the groundtruth range and bearing from the ego robot to the
     * other robots. */
    for (auto &ego_robot : (*robots_)) {

      /* The first element needs to create a new instance of the measurement.
       */
      bool first_entry{true};

      for (auto &other_agent : (*robots_)) {

        /* The robot cannot take any measurements of itself. */
        if (ego_robot.id == other_agent.id) {
          continue;
        }

        /* Calculate Groundtruth Range */
        const double x_difference{other_agent.groundtruth.states[k].x -
                                  ego_robot.groundtruth.states[k].x};

        const double y_difference{other_agent.groundtruth.states[k].y -
                                  ego_robot.groundtruth.states[k].y};

        const double range{std::sqrt(x_difference * x_difference +
                                     y_difference * y_difference)};

        /* If the range is larger than the max threshold it should not be
         * added to the list of measurements. */
        if (range > max_range) {
          continue;
        }

        /* Calculate the groundtruth bearings. */
        double bearing{std::atan2(y_difference, x_difference) -
                       ego_robot.groundtruth.states[k].orientation};

        /* Normalise the bearing between -180 and 180 (-pi and pi) */
        while (bearing >= M_PI)
          bearing -= 2.0 * M_PI;
        while (bearing < -M_PI)
          bearing += 2.0 * M_PI;

        /* According to the UTIAS multirobot localisation and mapping paper,
         * the robots have a field of view of 60 degrees (-0.52, 0.52
         * radians). Any bearing larger than that should not be included in
         * the measurements.
         */
        static const double max_fov{0.52};

        if (std::abs(bearing) > max_fov) {
          continue;
        }

        /* Populate data structure with the calculated measurement. */
        if (first_entry) {
          first_entry = false;
          ego_robot.groundtruth.measurements.emplace_back(
              ego_robot.groundtruth.states[k].time, other_agent.barcode, range,
              bearing);
        } else {
          Robot::Measurement &newest_measurement =
              ego_robot.groundtruth.measurements.back();

          newest_measurement.subjects.push_back(other_agent.barcode);
          newest_measurement.ranges.push_back(range);
          newest_measurement.bearings.push_back(bearing);
        }
      }

      /* Determine the groundtruth range and bearing from landmarks. */
      for (const auto landmark : (*landmarks_)) {

        double x_difference{landmark.x - ego_robot.groundtruth.states[k].x};
        double y_difference{landmark.y - ego_robot.groundtruth.states[k].y};
        double range{std::sqrt(x_difference * x_difference +
                               y_difference * y_difference)};

        /* If the range is larger than the max threshold it should not be
         * added to the list of measurements. */
        if (range > max_range) {
          continue;
        }

        double bearing = std::atan2(y_difference, x_difference) -
                         ego_robot.groundtruth.states[k].orientation;

        /* Normalise the orientation between -180 and 180 (-pi and pi) */
        while (bearing >= M_PI)
          bearing -= 2.0 * M_PI;
        while (bearing < -M_PI)
          bearing += 2.0 * M_PI;

        /* According to the UTIAS paper, the robots have a field of view of 60
         * degrees (-0.52, 0.52 radians). */
        if (std::abs(bearing) > 0.52) {
          continue;
        }

        /* Populate data structure with the calculated measurement. */
        if (first_entry) {
          first_entry = false;
          ego_robot.groundtruth.measurements.emplace_back(
              ego_robot.groundtruth.states[k].time, landmark.barcode, range,
              bearing);
        } else {
          ego_robot.groundtruth.measurements.back().subjects.push_back(
              landmark.barcode);
          ego_robot.groundtruth.measurements.back().ranges.push_back(range);
          ego_robot.groundtruth.measurements.back().bearings.push_back(bearing);
        }
      }
    }
  }
}

void Simulator::setGeneratorSeed(unsigned long seed) {

  /* If a seed is not provided, generate a random one. Otherwise, use the one
   * provided by the user. */
  if (seed == 0) {

    std::random_device random_device;
    seed = random_device();
  }

  generator_.seed(seed);
  std::cout << "\033[1;32mSimulator is using seed:\033[0m " << seed
            << std::endl;
}

/**
 * @brief Loop through measurments and adds Gaussian noise.
 */
void Simulator::addGaussianNoise() {

  /* Apply Gaussian noise to all measurements. */
  for (auto &robot : (*robots_)) {

    /* Check if all the erro variance have been set. */
    if (0.0 == robot.forward_velocity_error.variance ||
        0.0 == robot.angular_velocity_error.variance ||
        0.0 == robot.range_error.variance ||
        0.0 == robot.bearing_error.variance) {
      throw std::runtime_error("Error variances not set, Call "
                               "Simulator::setErrorStatistics before "
                               "calling this function");
    }

    /* Create Gaussian noise generators. */
    std::normal_distribution<double> forward_velocity_noise(
        0, std::sqrt(robot.forward_velocity_error.variance));

    std::normal_distribution<double> angular_velocity_noise(
        0, std::sqrt(robot.angular_velocity_error.variance));

    /* Apply Gaussian noise to odometry. */
    for (const auto &odometry : robot.groundtruth.odometry) {
      robot.synced.odometry.emplace_back(Robot::Odometry{
          .time = odometry.time,
          .forward_velocity =
              odometry.forward_velocity + forward_velocity_noise(generator_),
          .angular_velocity =
              odometry.angular_velocity + angular_velocity_noise(generator_)});
    }

    /* Apply Gaussian noise to range and bearing measurements. */
    std::normal_distribution<double> range_noise(
        robot.range_error.mean, std::sqrt(robot.range_error.variance));

    std::normal_distribution<double> bearing_noise(
        robot.bearing_error.mean, std::sqrt(robot.bearing_error.variance));

    /* For each measurment, add Gaussian noise. */
    for (const Robot::Measurement &measurement :
         robot.groundtruth.measurements) {

      /* Copy the measurement */
      robot.synced.measurements.push_back(measurement);

      /* Adding Gaussian noise to the measurements of all the subjects. */
      size_t total_measurements =
          robot.synced.measurements.back().subjects.size();

      for (unsigned short s{}; s < total_measurements; s++) {

        robot.synced.measurements.back().ranges[s] +=
            range_noise(this->generator_);

        robot.synced.measurements.back().bearings[s] +=
            bearing_noise(this->generator_);

        /* Normalise the bearing error. */
        while (robot.synced.measurements.back().bearings[s] >= M_PI)
          robot.synced.measurements.back().bearings[s] -= 2.0 * M_PI;
        while (robot.synced.measurements.back().bearings[s] < -M_PI)
          robot.synced.measurements.back().bearings[s] += 2.0 * M_PI;
      }
    }
  }
}
} // namespace Data
