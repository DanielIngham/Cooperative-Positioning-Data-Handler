#ifndef INCLUDE_INCLUDE_LANDMARK_H_
#define INCLUDE_INCLUDE_LANDMARK_H_

struct Landmark {
	int id;			///< Numerical identifier for the landmark. 
	int barcode;		///< Barcode associated with the landmark. This is what the robots will read during there operation to identify the landmarks.

	double x;		///< The landmark's golbal x-coordinate [m]
	double y;		///< The landmark's golbal y-coordinate [m]
	double x_std_dev;	///< The x-standard deviation of the positioning error [m]
	double y_std_dev;	///< The y-standard deviation of the positioning error [m]
};


#endif  // INCLUDE_INCLUDE_LANDMARK_H_
