# The plots are defined as: 
## .csv the matlab synced and interpolated output.
## 's'  the c++ synced and interpolated output. This should be identical to the Matlab output.
## 'r'  the raw data extracted from the dataset.

# Plot settings
set multiplot layout 3,1 title "Ground truth"
set xlabel "time [s]"
set grid
set key inside

# Set the x range. This was choosen specifically to showcase some interpolation issues.
#set xrange[1271.5:1272.2]
set xrange[22:28]
# Set up datafile seperator as both ',' (for csv files) and '	' (for .dat files).
set datafile sep ",	"

#### Ground truth plot
set title "Robot x Groundtruth Trajectory"
set ylabel "x Position [m]"
plot \
	"../data/MRCLAM_Dataset1/output/Groundtruth-State.dat" index 0 using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $2 : 1/0) with points pointsize 1.4 linecolor rgb "red" pointtype 7 title "Raw",\
	"" index 0 using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $2 : 1/0) with points pointsize 1.4 linecolor rgb "purple" title "Interpolated", \
	"../test/Matlab_output/Robot1_Groundtruth.csv" using 1:2 with points pointsize 1.4 linecolor rgb "blue" title "Matlab Interpolated"

set title "Robot y trajectory"
set ylabel "y Position [m]"
plot \
	"../data/MRCLAM_Dataset1/output/Groundtruth-State.dat" index 0 using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $3 : 1/0) with points pointsize 1.4 linecolor rgb "red" pointtype 7 title "Raw",\
	"" index 0 using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $3 : 1/0) with points pointsize 1.4 linecolor rgb "purple" title "Iterpolated", \
	"../test/Matlab_output/Robot1_Groundtruth.csv" using 1:3 with points pointsize 1.4 linecolor rgb "blue" title "Matlab Interpolated"

set title "Robot Orientation"
set ylabel "Orientation [rad]"
plot \
	"../data/MRCLAM_Dataset1/output/Groundtruth-State.dat" index 0 using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $4 : 1/0) with points pointsize 1.4 linecolor rgb "red" pointtype 7 title "Raw",\
	"" index 0 using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $4 : 1/0) with points pointsize 1.4 linecolor rgb "purple" title "Interpolated", \
	"../test/Matlab_output/Robot1_Groundtruth.csv"  using 1:4 with points pointsize 1.4 linecolor rgb "blue" title "Matlab Interpolated"

unset multiplot
pause -1 
