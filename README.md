# C++ UTIAS Multi-Robot Data Extractor
This project provides a c++ class as an interface for using the [UTIAS Multi-Robot Cooperative Localisation and Mapping dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/index.html). The project provides the following functionality:
- Extracts the UTIAS dataset into a c++ class allowing for easy interfacing with the dataset.
- Syncs the timesteps across all measuremets using the same approach as that of the [MATLAB Script](http://asrl.utias.utoronto.ca/datasets/mrclam/#Tools) provided with the dataset (linear interpolation).
- Calculates the corresponding sensor groundtruth for the odometry and measuremet sensors, using the provided state groundtruth (2D position and heading).
- Calculates the sensor error statistics used in Bayesian filtering frameworks.
- Provides a interface for [gnuplot](http://gnuplot.info/) to allow for visualisation of extracted data and calculated error statistics.
# Documentation 
For more information, the documentation for this project is available at: [Cooperative Positioning Data Handler github page.](https://danielingham.github.io/Cooperative-Positioning-Data-Handler/)
# Overview
