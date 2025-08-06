cmake_minimum_required(VERSION 3.10)
Project(GnuPlot_iostream)

if(POLICY CMP0167)
cmake_policy(SET CMP0167 NEW)
endif(POLICY CMP0167)

#options.
option(GnuPlotIostream_BuildTests "build gnuplot iostream tests" ON)
option(GnuPlotIostream_BuildExamples "build gnuplot iostream examples" ON)

# packages.
find_package(Boost REQUIRED COMPONENTS
    iostreams system filesystem
)

# Target.
add_library(gnuplot_iostream INTERFACE)

target_include_directories(gnuplot_iostream INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    $<INSTALL_INTERFACE:>)
target_link_libraries(gnuplot_iostream INTERFACE
    Boost::iostreams
    Boost::system
    Boost::filesystem
)
