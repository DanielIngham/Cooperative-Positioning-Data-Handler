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
 */
void Simulator::setRobotOdometry() {

  std::random_device rd;
  for (unsigned short id = 0; id < total_robots; id++) {

    for (unsigned long k = 0; k < data_points_ - 1; k++) {
    }
  }
}
