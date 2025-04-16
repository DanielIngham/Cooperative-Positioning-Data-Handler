/**
 * @file landmark.h
 * @brief Header file of the Landmark struct.  
 * @author Daniel Ingham
 * @date 2025-04-16
 */
#ifndef INCLUDE_INCLUDE_LANDMARK_H_
#define INCLUDE_INCLUDE_LANDMARK_H_

/**
 * @struct Landmark
 * @brief Houses all data related to a given landmark in the UTIAS multi-robot localisation and mapping dataset. 
 * @details This struct contains all the data found in the Landmarks.dat files. The DataExtractor::readLandmarks is responsible for populating this data structure with the appropriate values from a provided dataset.
 */
struct Landmark {
	int id;			///< Numerical identifier for the landmark. 
	int barcode;		///< Barcode associated with the landmark. This is what the robots will read during there operation to identify the landmarks.
	double x;		///< The landmark's golbal x-coordinate [m]
	double y;		///< The landmark's golbal y-coordinate [m]
	double x_std_dev;	///< The x-standard deviation of the positioning error [m]
	double y_std_dev;	///< The y-standard deviation of the positioning error [m]
};

#endif  // INCLUDE_INCLUDE_LANDMARK_H_
