# C++ UTIAS Multi-Robot Data Extractor
This project provides a c++ class as an interface for using the [UTIAS Multi-Robot Cooperative Localisation and Mapping dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/index.html). The project provides the following functionality:
- Extracts the UTIAS dataset into a c++ class allowing for easy interfacing with the dataset.
- Syncs the timesteps across all measurements using the same approach as that of the [MATLAB Script](http://asrl.utias.utoronto.ca/datasets/mrclam/#Tools) provided with the dataset (linear interpolation).
- Calculates the corresponding sensor groundtruth for the odometry and measurement sensors, using the provided state groundtruth (2D position and heading).
- Calculates the sensor error statistics used in Bayesian filtering frameworks.
- Provides an interface for [gnuplot](http://gnuplot.info/) to allow for visualisation of extracted data and calculated error statistics.

# Building the Data Handler
To build the data handler library, simply run the following commands:
```bash
mkdir build && cd build
cmake ..
cmake --build .
```
This builds the static library into the `./build` with the name `libdata_handler.a`.

# Including the library in your CMakeLists.txt 
To include the library your CMakeLists.txt:
```CMake
add_subdirectory([path/to/library] "LibBuild")

target_link_libraries(<target> PRIVATE DataHandler)
```

# Documentation 
For more information, the documentation for this project is available at: [Cooperative Positioning Data Handler GitHub page.](https://danielingham.github.io/Cooperative-Positioning-Data-Handler/)

# Examples
For example of how to use the data handler, see the constructor in [filter.cpp](https://github.com/DanielIngham/Cooperative-Positioning-Filters/blob/master/src/filter.cpp), from the [cooperative localisation filters repository](https://github.com/DanielIngham/Cooperative-Positioning-Filters.git).

