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

# Plot settings
set xlabel "time [s]"
set grid
set key inside

do for [i=1:5] {
	if (GPVAL_TERM ne "qt") {
		set output sprintf(plots_folder . "/Robot-%d-State-error." . file_type , i)
	}
	
	set multiplot layout 3,1 title sprintf("Robot %d State Estimation Error", i)

	set ylabel "|x-position error| [m]"
	plot data_folder . "/state_error.dat" index (i-1) using 1:(abs($2)) with linespoints pointsize 0.1 linecolor rgb "red" pointtype 7 notitle

	set ylabel "|y-position error| [m]"
	plot data_folder . "/state_error.dat" index (i-1) using 1:(abs($3)) with linespoints pointsize 0.1 linecolor rgb "red" pointtype 7 notitle

	set ylabel "|orientation error| [rad]"
	plot data_folder . "/state_error.dat" index (i-1) using 1:(abs($4)) with linespoints pointsize 0.1 linecolor rgb "red" pointtype 7 notitle

	unset multiplot

	pause pause_length
}


if (GPVAL_TERM ne "qt") {
	set output sprintf(plots_folder . "/Robot-State-error." . file_type , i)
}
set multiplot layout 3,1 title sprintf("All Robot State Estimation Error")
set key inside

set ylabel "|x-position error| [m]"
plot \
	data_folder . "/state_error.dat" index 0 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 title "Robot 1", \
	"" index 1 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 title "Robot 2",\
	"" index 2 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 title "Robot 3",\
	"" index 3 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 title "Robot 4",\
	"" index 4 using 1:(abs($2)) with linespoints pointsize 0.1 pointtype 7 title "Robot 5"

set ylabel "|y-position error| [m]"
plot \
	data_folder . "/state_error.dat" index 0 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 title "Robot 1", \
	"" index 1 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 title "Robot 2",\
	"" index 2 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 title "Robot 3",\
	"" index 3 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 title "Robot 4",\
	"" index 4 using 1:(abs($3)) with linespoints pointsize 0.1 pointtype 7 title "Robot 5"

set ylabel "|orientation error| [rad]"
plot \
	data_folder . "/state_error.dat" index 0 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 title "Robot 1", \
	"" index 1 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 title "Robot 2",\
	"" index 2 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 title "Robot 3",\
	"" index 3 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 title "Robot 4",\
	"" index 4 using 1:(abs($4)) with linespoints pointsize 0.1 pointtype 7 title "Robot 5"
unset multiplot
unset output

pause pause_length
