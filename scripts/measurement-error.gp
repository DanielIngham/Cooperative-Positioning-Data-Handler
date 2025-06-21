data_folder = dataset_directory
plots_folder = plots_directory

pause_length = 0

# Set plot to save to pdf output
set term (file_type eq "pdf") ? "pdfcairo" : \
        (file_type eq "png") ? "pngcairo" : \
        (file_type eq "svg") ? "svg" : "qt"

# Check current terminal is qt
if (GPVAL_TERM eq "qt") {
	pause_length = -1
} 

# Plot Settings
set xlabel "Time [s]"
set grid 
set key inside

# Save the plots for each Robot
do for [i=1:5] {

	##########################
	# Forward Velocity Input #
	##########################
	if (GPVAL_TERM ne "qt") {
		set output sprintf(plots_folder . "/Forward-Velocity/Robot-%d-Foward-Velocity-Error." . file_type , i)
	}

	plot \
		data_folder . "/Odometry-Error.dat" index (i-1) using 1:2 with points pointsize 0.1 notitle 

	pause pause_length

	##########################
	# Angular Velocity Input #
	##########################
	if (GPVAL_TERM ne "qt") {
		set output sprintf(plots_folder . "/Angular-Velocity/Robot-%d-Angular-Velocity-Error." . file_type , i)
	}

	plot data_folder . "/Odometry-Error.dat" index (i-1) using 1:3 with points pointsize 0.1 notitle 

	pause pause_length

	#####################
	# Range Measurement #
	#####################
	if (GPVAL_TERM ne "qt") {
		set output sprintf(plots_folder . "/Range/Robot-%d-Range-Error." . file_type , i)
	}

	plot data_folder . "/Measurement-Error.dat" index (i-1) using 1:3 with points pointsize 0.1 notitle 

	pause pause_length

	#######################
	# Bearing Measurement #
	#######################
	if (GPVAL_TERM ne "qt") {
		set output sprintf(plots_folder . "/Bearing/Robot-%d-Bearing-Error." . file_type , i)
	}

	plot data_folder . "/Measurement-Error.dat" index (i-1) using 1:4 with points pointsize 0.1 notitle

	pause pause_length

	unset multiplot
}
