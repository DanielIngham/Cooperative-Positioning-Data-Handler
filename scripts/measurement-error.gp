data_folder = dataset_directory
plots_folder = plots_directory

# Set plot to save to pdf output
set terminal pdfcairo enhanced font 'Helvetica,12'

# Plot Settings
set xlabel "Time [s]"
set grid 
set key inside

# Save the plots for each Robot
do for [i=1:5] {
	set output sprintf(plots_folder . "/Forward-Velocity/Robot-%d-Foward-Velocity-Error.pdf" , i)

	plot \
		data_folder . "/Odometry-Error.dat" index (i-1) using 1:2 with points pointsize 0.1 notitle 

	set output sprintf(plots_folder . "/Angular-Velocity/Robot-%d-Angular-Velocity-Error.pdf" , i)

	plot \
		data_folder . "/Odometry-Error.dat" index (i-1) using 1:3 with points pointsize 0.1 notitle 

	set output sprintf(plots_folder . "/Range/Robot-%d-Range-Error.pdf" , i)

	plot \
		data_folder . "/Measurement-Error.dat" index (i-1) using 1:3 with points pointsize 0.1 notitle 

	set output sprintf(plots_folder . "/Bearing/Robot-%d-Bearing-Error.pdf" , i)

	plot \
		data_folder . "/Measurement-Error.dat" index (i-1) using 1:4 with linespoints notitle
	unset multiplot
}
