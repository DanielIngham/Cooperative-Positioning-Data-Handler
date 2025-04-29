/**
 * The class simulates data in the same form as the UTIAS mulirobot localsiation
 * and mapping dataset.
 * @file simulator.cpp
 * @brief Class implementation file responsible for simulating the data for
 * multi-robot localisation and mapping.
 * @author Daniel Ingham
 * @date 2025-04-25
 */

#include "../include/simulator.h"
#include <random>
#include <stdexcept>
#include <string>

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
                     std::vector<unsigned short int> &barcodes)
    : data_points_(data_points), sample_period_(sample_period),
      robots_(&robots), landmarks_(&landmarks), barcodes_(&barcodes) {

  setSimulation(data_points_, sample_period_, robots, landmarks, barcodes);
}

/**
 * @brief Populates the robots and landmarks with simulated values.
 */
void Simulator::setSimulation(const unsigned long int data_points,
                              double sample_period, std::vector<Robot> &robots,
                              std::vector<Landmark> &landmarks,
                              std::vector<unsigned short int> &barcodes) {
  this->data_points_ = data_points;
  this->sample_period_ = sample_period;

  this->total_landmarks = landmarks.size();
  this->total_robots = robots.size();
  this->total_barcodes = this->total_landmarks + this->total_robots;

  this->robots_ = &robots;
  this->landmarks_ = &landmarks;
  this->barcodes_ = &barcodes;

  assignVectorMemory();
  setBarcodes();
  setLandmarks();
  setRobotsInitalState();
  setRobotOdometry();
}

/**
 * @brief Assigns the memory sizes for the vectors to be populated by the
 * simulator.
 */
void Simulator::assignVectorMemory() {
  for (unsigned short id = 0; id < this->total_robots; id++) {
    (*robots_)[id].groundtruth.states.reserve(this->data_points_);
    (*robots_)[id].synced.odometry.reserve(this->data_points_);

    /* Set the first element to the origin. This will be overwritten with random
     * values in Simulator::setRobots(). */
    (*robots_)[id].groundtruth.states.push_back(Robot::State(0, 0, 0, 0));
    (*robots_)[id].synced.odometry.push_back(Robot::Odometry(0, 0, 0));
  }
}

/**
 * @brief Sets the barcodes for each of the robots and landmarks.
 * @note The barcodes set in the simulator are the same as the ID. The only
 * reason the barcodes are set at all is for compatability with the original
 * UTIAS multirobot localisation and mapping dataset.
 */
void Simulator::setBarcodes() {
  for (unsigned short int id = 0; id < total_barcodes; id++) {
    (*barcodes_)[id] = id + 1;
  }
}

/**
 * @brief Sets the x and y coordinate for the number of landmarks provided.
 */
void Simulator::setLandmarks() {

  /* Random Setup and seeding. */
  std::random_device rd;
  std::mt19937 generator(rd());

  std::uniform_real_distribution<double> deviation(
      this->variance.landmarks[MIN], this->variance.landmarks[MAX]);

  /* Loop through each landmark and set the id and standard deviation */
  for (unsigned short i = 0; i < this->total_landmarks; i++) {

    /* Set the ID for each Landmark */
    (*landmarks_)[i].id = this->total_robots + (i + 1);
    /* Set the variance for each landmark */
    (*landmarks_)[i].x_std_dev = deviation(generator);
    (*landmarks_)[i].y_std_dev = deviation(generator);
  }

  /* Generate random x, y, angle and radius functions */
  std::uniform_real_distribution<double> position_x(0, this->limits_.width);
  std::uniform_real_distribution<double> position_y(0, this->limits_.height);

  /* Set the first landmark with a random x,y coordinate pair. */
  (*landmarks_)[0].x = position_x(generator);
  (*landmarks_)[0].y = position_y(generator);

  /* Loop through each landmark and assign a x,y coordinate that is at least 2m
   * apart from all other landmarks. */
  for (unsigned short i = 1; i < this->total_landmarks; i++) {
    /* Generate a random coordinate for the landmark*/
    (*landmarks_)[i].x = position_x(generator);
    (*landmarks_)[i].y = position_y(generator);

    for (unsigned short j = 0; i < j; j++) {

      /* Check that the new point is far enough away from other points randomly
       * choosen. */
      double x_difference = (*landmarks_)[i].x - (*landmarks_)[j].x;
      double y_difference = (*landmarks_)[i].y - (*landmarks_)[j].y;
      double distance =
          std::sqrt(x_difference * x_difference + y_difference * y_difference);

      /* If the point generated is too close to other points, restart the
       * process. */
      if (distance < 2.0) {
        i--;
        break;
      }
    }
  }
}

/**
 * @brief Sets the intial unique x,y coordinate and orienation for the number of
 * robots provided.
 */
void Simulator::setRobotsInitalState() {
  /* Random Setup and seeding. */
  std::random_device rd;
  std::mt19937 generator(rd());

  /* Set all the robot ID's and variance robots */
  std::uniform_real_distribution<double> forward_velocity_error(
      this->variance.forward_velocity[MIN],
      this->variance.forward_velocity[MAX]);
  std::uniform_real_distribution<double> angular_velocity_error(
      this->variance.angular_velocity[MIN],
      this->variance.angular_velocity[MAX]);

  std::uniform_real_distribution<double> range_error(this->variance.range[MIN],
                                                     this->variance.range[MAX]);
  std::uniform_real_distribution<double> bearing_error(
      this->variance.bearing[MIN], this->variance.bearing[MAX]);

  for (unsigned short id = 0; id < this->total_robots; id++) {
    (*robots_)[id].id = id + 1;

    (*robots_)[id].forward_velocity_error.variance =
        forward_velocity_error(generator);
    (*robots_)[id].angular_velocity_error.variance =
        angular_velocity_error(generator);

    (*robots_)[id].range_error.variance = range_error(generator);
    (*robots_)[id].bearing_error.variance = bearing_error(generator);
  }

  /* Set up random function for x and y position to fall within 1 metre of the
   * simulation limits. */
  std::uniform_real_distribution<double> position_x(1, this->limits_.width - 1);
  std::uniform_real_distribution<double> position_y(1,
                                                    this->limits_.height - 1);
  std::uniform_real_distribution<double> orienation(-M_PI, M_PI);

  /* Set the initial value for the robot 1 state. */
  bool unique;
  do {
    unique = true;

    /* Overwrite the origin values set in Robot::assignVectorMemory. */
    (*robots_)[0].groundtruth.states.at(0).time = 0;
    (*robots_)[0].groundtruth.states.at(0).x = position_x(generator);
    (*robots_)[0].groundtruth.states.at(0).y = position_y(generator),
    (*robots_)[0].groundtruth.states.at(0).orientation = orienation(generator);

    for (const auto &landmark : (*landmarks_)) {
      /* Check that the new point is far enough away from other points randomly
       * choosen. */
      double x_difference =
          landmark.x - (*robots_)[0].groundtruth.states.front().x;
      double y_difference =
          landmark.y - (*robots_)[0].groundtruth.states.front().y;
      double distance =
          std::sqrt(x_difference * x_difference + y_difference * y_difference);

      /* If the point generated is too close to other points, restart the
       * process. */
      if (distance < 2.0) {
        unique = false;
        break;
      }
    }
  } while (!unique);

  /* Set the initial values up for the remaining robots. */
  for (unsigned short id = 1; id < this->total_robots; id++) {

    /* Overwrite the origin values set in Robot::assignVectorMemory. */
    unique = true;
    (*robots_)[id].groundtruth.states.at(0).time = 0;
    (*robots_)[id].groundtruth.states.at(0).x = position_x(generator);
    (*robots_)[id].groundtruth.states.at(0).y = position_y(generator);
    (*robots_)[id].groundtruth.states.at(0).orientation = orienation(generator);

    /* Check that the position is far enough away from other robots */
    for (unsigned short j = 0; j < id; j++) {

      /* Check that the new point is far enough away from other points randomly
       * choosen. */
      double x_difference = (*robots_)[id].groundtruth.states.front().x -
                            (*robots_)[j].groundtruth.states.front().x;
      double y_difference = (*robots_)[id].groundtruth.states.front().y -
                            (*robots_)[j].groundtruth.states.front().y;
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
      /* Check that the new point is far enough away from other points randomly
       * choosen. */
      double x_difference =
          landmark.x - (*robots_)[id].groundtruth.states.front().x;
      double y_difference =
          landmark.y - (*robots_)[id].groundtruth.states.front().y;
      double distance =
          std::sqrt(x_difference * x_difference + y_difference * y_difference);

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
 * @brief Sets the odometry values (forward and angular velocity) for the number
 * of robots provided.
 * @note Simulator::setRobotsInitalState needs to be called before this
 * function. If this is not done, a std::runtime_error will be thrown.
 */
void Simulator::setRobotOdometry() {

  /* Create a centre point. */
  double centre_x = (this->limits_.width) / 2.0,
         centre_y = (this->limits_.height) / 2.0;

  /* Set up random number generator. */
  std::random_device rd;
  std::mt19937 generator(rd());

  /* Set up random number generation functions.
   * The walk length denotes the number of samples for which an input is
   * applied.
   */
  std::uniform_int_distribution<unsigned short> walk_length(20U, 50U);
  std::uniform_real_distribution<double> forward_velocity(
      0.0, limits_.forward_velocity);
  std::uniform_real_distribution<double> angular_velocity(
      -limits_.angular_velocity, limits_.angular_velocity);

  std::uniform_real_distribution<double> forward_change(-0.05, 0.05);
  std::uniform_real_distribution<double> angular_change(-0.1, 0.1);

  for (unsigned short id = 0; id < total_robots; id++) {
    if ((*robots_)[id].groundtruth.states.empty()) {
      throw std::runtime_error(
          "The initial state of Robot " + std::to_string(id + 1) +
          " was not set. Call Simulator::setRobotsInitalState before calling "
          "Simulator::setRobotOdometry");
    }
    /* Populate the robot's inital input. */
    (*robots_)[id].groundtruth.odometry.push_back(Robot::Odometry(
        0, forward_velocity(generator), angular_velocity(generator)));

    /* Calculate the resulting state from this those intpus */
    double x_position =
        (*robots_)[id].groundtruth.states.at(0).x +
        (*robots_)[id].groundtruth.odometry.at(0).forward_velocity *
            this->sample_period_ *
            std::cos((*robots_)[id].groundtruth.states.at(0).orientation);

    double y_position =
        (*robots_)[id].groundtruth.states.at(0).y +
        (*robots_)[id].groundtruth.odometry.at(0).forward_velocity *
            this->sample_period_ *
            std::sin((*robots_)[id].groundtruth.states.at(0).orientation);

    double orienation =
        (*robots_)[id].groundtruth.states.at(0).orientation +
        this->sample_period_ *
            (*robots_)[id].groundtruth.odometry.at(0).angular_velocity;

    (*robots_)[id].groundtruth.states.push_back(
        Robot::State(this->sample_period_, x_position, y_position, orienation));

    /* Assign a random walk length at random  */
    unsigned short random_walk_duration = walk_length(generator);

    /* Generate random odometry inputs for every datapoint. */
    for (unsigned long k = 1; k < this->data_points_; k++) {
      double angular_adjustment = 0.0;
      double forward_adjustment = 0.0;

      /* If the robot is within of the boundaries, the robot should be guided
       * back towards the centre of the simulation area. */
      if ((*robots_)[id].groundtruth.states.at(k).x < 1 ||
          (*robots_)[id].groundtruth.states.at(k).x > limits_.width - 1 ||
          (*robots_)[id].groundtruth.states.at(k).y < 1 ||
          (*robots_)[id].groundtruth.states.at(k).y > limits_.height - 1) {

        /* Calculate the distance from the centre points and get the angle
         * adjustment. */
        double x_difference =
            centre_x - (*robots_)[id].groundtruth.states.at(k).x;
        double y_difference =
            centre_y - (*robots_)[id].groundtruth.states.at(k).y;
        double bearing_for_centre =
            std::atan2(y_difference, x_difference) -
            (*robots_)[id].groundtruth.states.at(k).orientation;

        /* Normalise the orientation */
        while (bearing_for_centre >= M_PI)
          bearing_for_centre -= 2.0 * M_PI;
        while (bearing_for_centre <= -M_PI)
          bearing_for_centre += 2.0 * M_PI;

        /* If the the bearinging from the centre point is less than
         * approximately 10 degrees positive or negative, there is no need to
         * make further adjustments.  */
        if (std::abs(bearing_for_centre) > 0.2) {
          /* Gradually correct the orientation through adjustments to the
           * angular velocity. */
          /* NOTE: the an adjustement to forward velocity is not made. */
          if (bearing_for_centre > 0.0) {
            angular_adjustment = 0.02;
          } else {
            angular_adjustment = 0.02;
          }
        }
      } else if ((k % random_walk_duration) == 0) {
        /* Assign a new velocity adjustment */
        forward_adjustment = forward_change(generator);
        angular_adjustment = angular_change(generator);

        /* Assign a new random walk length at random  */
        random_walk_duration = walk_length(generator);
      }

      /* Boundary checks on new odometry values. */
      double new_forward_velocity =
          (*robots_)[id].groundtruth.odometry.at(k - 1).forward_velocity +
          forward_adjustment;
      double new_angular_velocity =
          (*robots_)[id].groundtruth.odometry.at(k - 1).angular_velocity +
          angular_adjustment;

      /* NOTE: it is assumed that the robots cannot reverse. */
      if (new_forward_velocity > limits_.forward_velocity)
        new_forward_velocity = limits_.forward_velocity;
      else if (new_forward_velocity < 0.0)
        new_forward_velocity = 0.0;

      if (new_angular_velocity > limits_.angular_velocity)
        new_angular_velocity = limits_.angular_velocity;
      else if (new_angular_velocity < -limits_.angular_velocity)
        new_angular_velocity = -limits_.angular_velocity;

      /* Populate odometry with new values. */
      (*robots_)[id].groundtruth.odometry.push_back(
          Robot::Odometry(this->sample_period_ * k, new_forward_velocity,
                          new_angular_velocity));

      /* Prevents the groundtruth from having one more value than the odometry.
       */
      if ((*robots_)[id].groundtruth.states.size() == this->data_points_) {
        continue;
      }
      /* Calculate the resulting state from this those intpus */
      x_position =
          (*robots_)[id].groundtruth.states.at(k).x +
          (*robots_)[id].groundtruth.odometry.at(k).forward_velocity *
              this->sample_period_ *
              std::cos((*robots_)[id].groundtruth.states.at(k).orientation);

      y_position =
          (*robots_)[id].groundtruth.states.at(k).y +
          (*robots_)[id].groundtruth.odometry.at(k).forward_velocity *
              this->sample_period_ *
              std::sin((*robots_)[id].groundtruth.states.at(k).orientation);

      orienation =
          (*robots_)[id].groundtruth.states.at(k).orientation +
          this->sample_period_ *
              (*robots_)[id].groundtruth.odometry.at(k).angular_velocity;

      (*robots_)[id].groundtruth.states.push_back(Robot::State(
          this->sample_period_, x_position, y_position, orienation));
    }
    std::cout << "Robot " << id + 1 << ": "
              << (*robots_)[id].groundtruth.odometry.size() << " : "
              << (*robots_)[id].groundtruth.states.size() << std::endl;
  }
}
