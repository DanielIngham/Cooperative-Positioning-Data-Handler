/**
 * The class simulates data in the same form as the UTIAS mulirobot localsiation and mapping dataset.
 * @file simulator.cpp
 * @brief Class implementation file responsible for simulating the data for multi-robot localisation and mapping.
 * @author Daniel Ingham
 * @date 2025-04-25
 */

#include "../include/simulator.h"
#include <cmath>
#include <random>

/*
 * @brief Default destructor.
 */
Simulator::~Simulator() {
}

/**
 * @brief constructor which populates the data for the robots and landmarks.
 */
Simulator::Simulator(const unsigned long int data_points, double sample_period, std::vector<Robot>& robots, std::vector<Landmark>& landmarks): data_points_(data_points), sample_period_(sample_period), robots_(robots), landmarks_(landmarks) {
	
}

double Simulator::distance(const Point& a, const Point& b) {
	float x_difference = a.x - b.x;
	float y_difference = a.y - b.y;
	return std::sqrt(x_difference * x_difference + y_difference * y_difference);
}
/**
 * @brief Sets the x and y coordinate of the landmarks provided.
 * @details Uses the Bridson algorithm to uniformly assign coordinates to landmarks in the simulation area.
 */
void Simulator::setLandmarks() {
	const unsigned short int k = 30U;

	/* Steup Grid */
	double r = 1.0f;
	double cell_size = r / std::sqrt(2);
	unsigned int grid_width = std::ceil(this->limits_.width / cell_size);
	unsigned int grid_height = std::ceil(this->limits_.height / cell_size);

	/* Initialise Data Structors */
	std::vector<Point> grid(grid_width * grid_height, {-1, 1});
	std::vector<Point> process_list;
	std::vector<Point> samples;

	/* Random Setup and seeding. */
	std::random_device rd;
	std::mt19937 generator(rd());

	/* Generate random x, y, angle and radius functions */
	std::uniform_real_distribution<double> disc_x(0, this->limits_.width);
	std::uniform_real_distribution<double> disc_y(0, this->limits_.height);
	std::uniform_real_distribution<double> disc_angle(0, 2 * M_PI);
	std::uniform_real_distribution<double> disc_radius(r, 2*r);

	/* Initial Point */
	Point first = {disc_x(generator), disc_y(generator)};
	process_list.push_back(first);
	samples.push_back(first);
	grid[(int)(first.y / cell_size) * grid_width + (int)(first.x / cell_size)] = first;

	/* Keep expanding until no more valid points can be added. */
	while(!process_list.empty()) {
		/* Choose a random point from the processList to try and place points around it. */
		int index = std::uniform_int_distribution<>(0, process_list.size() - 1)(generator);
		Point p = process_list[index];

		bool found = false;

		/* Try to add new points around the choosen point. */
		for (unsigned int i = 0; i < k; ++i) {
			float angle = disc_angle(generator);
			float radius = disc_radius(generator);

			Point newP = {
				p.x + radius * std::cos(angle),
				p.y + radius * std::sin(angle)
			};

			/* Skip if the new point falls outside the limits of the simulation. */
			if (newP.x < 0 || newP.y < 0 || newP.x >= this->limits_.width || newP.y >= this->limits_.height) 
				continue;
			

			int gx = newP.x / cell_size;
			int gy = newP.y / cell_size;
			bool tooClose = false;

			for (int dx = -2;  dx <= 2; ++dx) {
				for (int dy = -2; dy <= 2; ++dy) {
					int nx = gx + dx;
					int ny = gy + dy;

					if (nx < 0 || ny < 0 || nx >= grid_width || ny >= grid_height) 
						continue;
					
					Point neighbour = grid[ny * grid_width + nx];

					if (neighbour.x != -1 && distance(newP, neighbour) < r) {
						tooClose = true;
						break;
					}
				}

				if (tooClose) 
					break;
				
			}
			
			if (!tooClose) {
				process_list.push_back(newP);
				samples.push_back(newP);
			}
		}
	}
}
