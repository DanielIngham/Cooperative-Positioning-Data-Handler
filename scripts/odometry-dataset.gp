# The plots are defined as: 
## .csv the matlab synced and interpolated output.
## 's'  the c++ synced and interpolated output. This should be identical to the Matlab output.
## 'r'  the raw data extracted from the dataset.

# Plot settings
set multiplot layout 2,1 title "Robot 1 Odometry"
set xlabel "time [s]"
set grid
set key inside
set xrange[662:665]

# Set up datafile seperator as both ',' (for csv files) and '	' (for .dat files).
set datafile sep ",	"

set title "Robot Forward Velocity"
set ylabel "Forward velocity [m/s]"
plot \
	"../data/MRCLAM_Dataset1/output/Odometry.dat" index 0 using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $2 : 1/0) with points pointsize 1.4 linecolor rgb "red" pointtype 7 title "Raw",\
	"" index 0 using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $2 : 1/0) with points pointsize 1.4 title "Interpolated",\
	"../test/Matlab_output/Robot1_Odometry.csv" using 1:2 with points pointsize 1.4 title "Matlab Interpolated"

set title "Robot Angular Velocity"
set ylabel "Angular velocity [rad/s]"
plot \
	"../data/MRCLAM_Dataset1/output/Odometry.dat" index 0 using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $3 : 1/0) with points pointsize 1.4 linecolor rgb "red" pointtype 7 title "Raw",\
	"" index 0 using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $3 : 1/0) with points pointsize 1.4 title "Interpolated", \
	"../test/Matlab_output/Robot1_Odometry.csv" using 1:3 with points pointsize 1.4 title "Matlab Interpolated" 

unset multiplot
pause -1

