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

# Set the x range. This was choosen specifically to showcase some interpolation issues.
# Set up datafile seperator as both ',' (for csv files) and '	' (for .dat files).
set datafile sep ",	"

#### Ground truth plot
do for [i=1:5] {

	# Set the output file 
	set output sprintf(plots_folder . "/State/Robot-%d-State." . file_type , i)
	set multiplot layout 3,1 title sprintf("Robot %d Groundtruth Trajectory", i)

	set ylabel "x-position [m]"
	plot \
		data_folder . "/Groundtruth-State.dat" index (i-1) using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $2 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw",\
		"" index (i-1) using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $2 : 1/0) with points pointsize 0.1 linecolor rgb "purple" title "Interpolated"

	set ylabel "y-position [m]"
	plot \
		data_folder . "/Groundtruth-State.dat" index (i-1) using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $3 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw", \
		"" index (i-1) using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $3 : 1/0) with points pointsize 0.1 linecolor rgb "purple" title "Interpolated"

	set ylabel "orientation [rad]"
	plot \
		data_folder . "/Groundtruth-State.dat" index (i-1) using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $4 : 1/0) with points pointsize 0.1 linecolor rgb "red" pointtype 7 title "Raw", \
		"" index (i-1) using (stringcolumn(5) eq "s" ? $1 : 1/0):(stringcolumn(5) eq "s" ? $4 : 1/0) with points pointsize 0.1 linecolor rgb "purple" title "Interpolated"

	unset multiplot
	unset output
	

	####################
	# Plotting xy coordinates 
	# alongside landmarks
	####################
	set output sprintf(plots_folder . "/State/Robot-%d-Position." . file_type, i)
	plot \
		data_folder . "/Groundtruth-State.dat" index (i-1) using (stringcolumn(5) eq "s" ? $2 : 1/0):(stringcolumn(5) eq "s" ? $3 : 1/0) with linespoints pointsize 0.1 title "Robot Coordinate",\
		data_folder . "/landmarks.dat" using 3:4 with points pointsize 1.0 pointtype 6 title "Landmarks"
}
