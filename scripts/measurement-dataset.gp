data_folder = dataset_directory
plots_folder = plots_directory

# Set plot to save to pdf output
set term (file_type eq "pdf") ? "pdfcairo" : \
        (file_type eq "png") ? "pngcairo" : \
        (file_type eq "svg") ? "svg" : qt

# Plot settings
set xlabel "time [s]"
set grid
set key inside

# set xrange [750:800]

# Set up datafile seperator as both ',' (for csv files) and '	' (for .dat files).
set datafile sep ",	"

# Create a plot for every robot
do for [i=1:5] {
	#########################
	# Forward Velocity  Error
	#########################
	set output sprintf(plots_folder . "/Forward-Velocity/Robot-%d-Foward-Velocity." . file_type , i)
	set title sprintf("Robot %d Forward Velocity", i)
	set ylabel "Forward velocity [m/s]"
	plot \
		data_folder . "/Odometry.dat" index (i-1) using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $2 : 1/0) with points pointsize 0.1 title "Interpolated",\
		data_folder . "/Odometry.dat" index (i-1) using (stringcolumn(4) eq "g" ? $1 : 1/0):(stringcolumn(4) eq "g" ? $2 : 1/0) with points pointsize 0.1 title "Groundtruth",\
		"" index (i-1) using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $2 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw"

	#########################
	# Angular Velocity  Error
	#########################
	set output sprintf(plots_folder . "/Angular-Velocity/Robot-%d-Angular-Velocity." . file_type , i)
	set title sprintf("Robot %d Angular Velocity", i)
	set ylabel "Angular velocity [rad/s]"
	plot \
		data_folder . "/Odometry.dat" index (i-1) using (stringcolumn(4) eq "s" ? $1 : 1/0):(stringcolumn(4) eq "s" ? $3 : 1/0) with points pointsize 0.1 title "Interpolated", \
		data_folder . "/Odometry.dat" index (i-1) using (stringcolumn(4) eq "g" ? $1 : 1/0):(stringcolumn(4) eq "g" ? $3 : 1/0) with points pointsize 0.1 title "Groundtruth",\
		"" index (i-1) using (stringcolumn(4) eq "r" ? $1 : 1/0):(stringcolumn(4) eq "r" ? $3 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw"

	#########################
	# Range Measurement
	#########################
	set output sprintf(plots_folder . "/Range/Robot-%d-Range." . file_type , i)
	set title sprintf("Robot %d Range Measurements", i)
	set ylabel "Range [m]"
	plot \
		data_folder . "/Measurement.dat" index (i-1) using (stringcolumn(5) eq "g" ? $1 : 1/0):(stringcolumn(5) eq "g" ? $3 : 1/0) with points pointsize 0.1 title "Groundtruth",\
		"" index (i-1) using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $3 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw",\
		"" index (i-1) using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $3 : 1/0) with points pointsize 0.1 title "Interpolated"

	#########################
	# Bearing Measurement
	#########################
	set output sprintf(plots_folder . "/Bearing/Robot-%d-Bearing." . file_type , i)
	set title sprintf("Robot %d Bearing Measurements", i)
	set ylabel "Bearing [rad]"
	plot \
		data_folder . "/Measurement.dat" index (i-1) using (stringcolumn(5) eq "g" ? $1 : 1/0):(stringcolumn(5) eq "g" ? $4 : 1/0) with points pointsize 0.1 title "Groundtruth",\
		"" index (i-1) using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $4 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw",\
		"" index (i-1) using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $4 : 1/0) with points pointsize 0.1 title "Interpolated"
}
