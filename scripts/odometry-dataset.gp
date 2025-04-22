# Set plot to save to pdf output
set terminal pdfcairo enhanced font 'Helvetica,12'

data_folder = dataset_directory
plots_folder = plots_directory

# The plots are defined as: 
## .csv the matlab synced and interpolated output.
## 's'  the c++ synced and interpolated output. This should be identical to the Matlab output.
## 'r'  the raw data extracted from the dataset.

# Plot settings
set xlabel "time [s]"
set grid
set key inside

# Set up datafile seperator as both ',' (for csv files) and '	' (for .dat files).
set datafile sep ",	"



# Create a plot for every robot
do for [i=1:5] {
	set output sprintf(plots_folder . "/Forward-Velocity/Robot-%d-Foward-Velocity.pdf" , i)
	set title sprintf("Robot %d Forward Velocity", i)
	set ylabel "Forward velocity [m/s]"
	plot \
		data_folder . "/Odometry.dat" index (i-1) using (stringcolumn(4) eq "g" ? $1 : 1/0):(stringcolumn(4) eq "g" ? $2 : 1/0) with points pointsize 0.1 title "Groundtruth",\
		"" index (i-1) using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $2 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw",\
		"" index (i-1) using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $2 : 1/0) with points pointsize 0.1 title "Interpolated"

	set output sprintf(plots_folder . "/Angular-Velocity/Robot-%d-Angular-Velocity.pdf" , i)
	set title sprintf("Robot %d Angular Velocity", i)
	set ylabel "Angular velocity [rad/s]"
	plot \
		data_folder . "/Odometry.dat" index (i-1) using (stringcolumn(4) eq "g" ? $1 : 1/0):(stringcolumn(4) eq "g" ? $3 : 1/0) with points pointsize 0.1 title "Groundtruth",\
		"" index (i-1) using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $3 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw",\
		"" index (i-1) using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $3 : 1/0) with points pointsize 0.1 title "Interpolated"
}
