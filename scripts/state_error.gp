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

do for [i=1:5] {
	set output sprintf(plots_folder . "/Robot-%d-State-error." . file_type , i)
	
	set multiplot layout 3,1 title sprintf("Robot %d State Estimation Error", i)

	set ylabel "|x-position error| [m]"
	plot data_folder . "/state_error.dat" index (i-1) using 1:(abs($2)) with linespoints pointsize 0.1 linecolor rgb "red" pointtype 7 notitle

	set ylabel "|y-position error| [m]"
	plot data_folder . "/state_error.dat" index (i-1) using 1:(abs($3)) with linespoints pointsize 0.1 linecolor rgb "red" pointtype 7 notitle

	set ylabel "|orientation error| [rad]"
	plot data_folder . "/state_error.dat" index (i-1) using 1:(abs($4)) with linespoints pointsize 0.1 linecolor rgb "red" pointtype 7 notitle

	unset multiplot
}

set output sprintf(plots_folder . "/Robot-State-error." . file_type , i)
set multiplot layout 3,1 title sprintf("Robot %d State Estimation Error", i)

set ylabel "|x-position error| [m]"
plot \
	data_folder . "/state_error.dat" index 0 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle, \
	"" index 1 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 2 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 3 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 4 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle

set ylabel "|y-position error| [m]"
plot \
	data_folder . "/state_error.dat" index 0 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle, \
	"" index 0 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 1 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 2 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 3 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 4 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 notitle

set ylabel "|orientation error| [rad]"
plot \
	data_folder . "/state_error.dat" index 0 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 notitle, \
	"" index 0 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 1 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 2 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 3 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 notitle,\
	"" index 4 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 notitle
unset multiplot
unset output
