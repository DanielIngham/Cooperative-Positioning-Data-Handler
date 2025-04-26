/**
 * The class simulates data in the same form as the UTIAS mulirobot localsiation and mapping dataset.
 * @file simulator.cpp
 * @brief Class implementation file responsible for simulating the data for multi-robot localisation and mapping.
 * @author Daniel Ingham
 * @date 2025-04-25
 */

#include "../include/simulator.h"


/**
 * @brief Default constructor.
 */
Simulator::Simulator() {
}
/*
 * @brief Default destructor.
 */
Simulator::~Simulator() {
}

/**
 * @brief Constructor which populates the data for the robots and landmarks.
 */
Simulator::Simulator(const unsigned long int data_points, double sample_period, std::vector<Robot>& robots, std::vector<Landmark>& landmarks,  std::vector<unsigned short int>& barcodes): data_points_(data_points) ,sample_period_(sample_period), robots_(&robots), landmarks_(&landmarks), barcodes_(&barcodes) {

	setSimulation(data_points_, sample_period_, robots, landmarks, barcodes);
}

/**
 * @brief Populates the robots and landmarks with simulated values.
 */
void Simulator::setSimulation(const unsigned long int data_points, double sample_period, std::vector<Robot>& robots, std::vector<Landmark>& landmarks, std::vector<unsigned short int>& barcodes) {
	this->data_points_ = data_points;
	this->sample_period_ = sample_period;

	this->total_landmarks = landmarks.size();
	this->total_robots = robots.size();
	this->total_barcodes = this->total_landmarks + this->total_robots;

	this->robots_ = &robots;
	this->landmarks_ = &landmarks;
	this->barcodes_ = &barcodes;

	setBarcodes();
	setLandmarks();
}

/**
 * @brief Sets the barcodes for each of the robots and landmarks. 
 * @note The barcodes set in the simulator are the same as the ID. The only reason the barcodes are set at all is for compatability with the original UTIAS multirobot localisation and mapping dataset. 
 */
void Simulator::setBarcodes() {
	for (unsigned short int id = 0; id < total_barcodes; id++) {
		(*barcodes_)[id] = id + 1;
	}
}

/**
 * @brief Sets the x and y coordinate of the landmarks provided.
 */
void Simulator::setLandmarks() {

	/* Random Setup and seeding. */
	std::random_device rd;
	std::mt19937 generator(rd());
	
	std::uniform_real_distribution<double> deviation(this->variance.landmarks[MIN], this->variance.landmarks[MAX]);
	
	for (unsigned short int i = 0; i < this->total_landmarks; i++) {
		
		/* Set the ID for each Landmark */
		(*landmarks_)[i].id = this->total_robots + (i + 1);
		/* Set the variance for each landmark */ 
		(*landmarks_)[i].x_std_dev = deviation(generator);
		(*landmarks_)[i].y_std_dev = deviation(generator);

		std::cout << "Landmark " << (*landmarks_)[i].id<< ": " << (*landmarks_)[i].x_std_dev << ", " << (*landmarks_)[i].y_std_dev << std::endl;
	}

	/* Generate random x, y, angle and radius functions */
	std::uniform_real_distribution<double> disc_x(0, this->limits_.width);
	std::uniform_real_distribution<double> disc_y(0, this->limits_.height);
	
	/* Set the first landmark with a random x,y coordinate pair. */
	(*landmarks_)[0].x = disc_x(generator);
	(*landmarks_)[0].y = disc_y(generator);

	for (unsigned short int i = 1; i < this->total_landmarks; i++) {
		/* Generate a random coordinate for the landmark*/
		(*landmarks_)[i].x = disc_x(generator);
		(*landmarks_)[i].y = disc_y(generator);

		for (unsigned short int j = 0; i < j; i++) {

			/* Check that the new point is far enough away from other points randomly choosen. */
			double x_difference = (*landmarks_)[i].x - (*landmarks_)[j].x;
			double y_difference = (*landmarks_)[i].y - (*landmarks_)[j].y;
			double distance = std::sqrt(x_difference*x_difference + y_difference*y_difference);

			/* If the point generated is too close to other points, restart the process. */
			if (distance < 2.0) {
				i--;
				break;
			}
		}
	}
}

