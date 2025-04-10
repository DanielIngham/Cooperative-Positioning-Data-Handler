# The plots are defined as: 
## .csv the matlab synced and interpolated output.
## 's'  the c++ synced and interpolated output. This should be identical to the Matlab output.
## 'r'  the raw data extracted from the dataset.

# Plot settings
set xlabel "time [s]"
set grid
set key inside

set multiplot layout 3,1 title "Ground truth"
# Set up datafile seperator as both ',' (for csv files) and '	' (for .dat files).
set datafile sep ",	"

# Set the x range. This was choosen specifically to showcase some interpolation issues.
# set xrange[1271.5:1272.5]
# set xrange[145.42:145.62]
# set xrange [642.02: 642.10]

# Ground truth plot
set title "Robot x Groundtruth Trajectory"
set ylabel "x Position [m]"
plot "./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $2 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Groundtruth.csv" using 1:2 with points pointsize 0.4 title "Matlab Interpolated", \
	"./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $2 : 1/0) with points pointsize 0.4 title "Interpolated"

set title "Robot y trajectory"
set ylabel "y Position [m]"
plot "./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $3 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Groundtruth.csv" using 1:3 with points pointsize 0.4 title "Matlab Interpolated", \
	"./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $3 : 1/0) with points pointsize 0.4 title "Iterpolated"

set title "Robot Orientation"
set ylabel "Orientation [rad]"
plot "./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $4 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Groundtruth.csv" using 1:4 with points pointsize 0.4 title "Matlab Interpolated", \
	"./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $4 : 1/0) with points pointsize 0.4 title "Interpolated"
unset multiplot
pause -1 

# Robot Odometry plot
set multiplot layout 2,1 title "Robot 1 Odometry"
set datafile sep ",	"
#set xrange[600:700]
set xrange [642.02: 642.10]

set title "Robot Forward Velocity"
set ylabel "Forward velocity [m/s]"
plot "./data/robot0-Odometry.dat" using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $2 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Odometry.csv" using 1:2 with points pointsize 0.4 title "Matlab Interpolated", \
	"./data/robot0-Odometry.dat" using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $2 : 1/0) with points pointsize 0.4 title "Interpolated"

set title "Robot Angular Velocity"
set ylabel "Angular velocity [rad/s]"
plot "./data/robot0-Odometry.dat" using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $3 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Odometry.csv" using 1:3 with points pointsize 0.4 title "Matlab Interpolated", \
	"./data/robot0-Odometry.dat" using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $3 : 1/0) with points pointsize 0.4 title "Interpolated"
pause -1

# Robot Measurement plot
set multiplot layout 3,1 title "Robot 1 Measurement"
unset xrange

set title "Other Robots Barcodes"
set ylabel "Subjects"
plot "./data/robot0-Meaurement.dat" using (stringcolumn(5) eq 'r' ? $1: 1/0):(stringcolumn(5) eq 'r' ? $2 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Measurement.csv" using 1:2 with points pointsize 0.4 linecolor rgb "red" pointtype 7 title "Matlab Interpolated", \
	"./data/robot0-Meaurement.dat" using (stringcolumn(5) eq 's' ? $1: 1/0):(stringcolumn(5) eq 's' ? $2 : 1/0) with points pointsize 0.4 title "Interpolated" 

set title "Other Robots Ranges"
set ylabel "Ranges"
plot "./data/robot0-Meaurement.dat" using (stringcolumn(5) eq 'r' ? $1: 1/0):(stringcolumn(5) eq 'r' ? $3 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Measurement.csv" using 1:3 with points pointsize 0.4 linecolor rgb "red" pointtype 7 title "Matlab Interpolated", \
	"./data/robot0-Meaurement.dat" using (stringcolumn(5) eq 's' ? $1: 1/0):(stringcolumn(5) eq 's' ? $3 : 1/0) with points pointsize 0.4 title "Interpolated" 

set title "Other Robots Bearings"
set ylabel "Bearings"
plot "./data/robot0-Meaurement.dat" using (stringcolumn(5) eq 'r' ? $1: 1/0):(stringcolumn(5) eq 'r' ? $4 : 1/0) with points pointsize 0.8 linecolor rgb "red" pointtype 7 title "Raw",\
	"./Matlab_output/Robot1_Measurement.csv" using 1:4 with points pointsize 0.4 linecolor rgb "red" pointtype 7 title "Matlab Interpolated", \
	"./data/robot0-Meaurement.dat" using (stringcolumn(5) eq 's' ? $1: 1/0):(stringcolumn(5) eq 's' ? $4 : 1/0) with points pointsize 0.4 title "Interpolated" 

pause -1

set multiplot layout 2,1 title "Odometry Error"
set title "Forward Velocity Error"
set ylabel "Error [m/s]"
plot "./data/robot0-Odometry-Error.dat" using 1:2 with linespoints pointsize 0.2

set title "Angular Velocity Error"
set ylabel "Error [rad/s]"
plot "./data/robot0-Odometry-Error.dat" using 1:3 with linespoints pointsize 0.2
pause -1
