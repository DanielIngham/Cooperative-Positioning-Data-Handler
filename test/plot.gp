# Plot settings
set title "Robot Trajectories"
set xlabel "X Position"
set ylabel "Y Position"
set grid
set key outside


set multiplot layout 3,1 title "Robot 1"
set datafile sep ",	"
set xrange[1271.5:1272.5]

plot "./Matlab_output/Robot1_Groundtruth.csv" using 1:2 with points pointsize 0.4 title "x-intepolated", \
	"./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "g" ? $1 : 1/0):(stringcolumn(5) eq "g" ? $2 : 1/0) with points pointsize 0.4 title "x"

plot "./Matlab_output/Robot1_Groundtruth.csv" using 1:3 with points pointsize 0.4 title "y-intepolated", \
	"./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "g" ? $1 : 1/0):(stringcolumn(5) eq "g" ? $3 : 1/0) with points pointsize 0.4 title "y"

plot "./Matlab_output/Robot1_Groundtruth.csv" using 1:4 with points pointsize 0.4 title "orientation-intepolated", \
	"./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "g" ? $1 : 1/0):(stringcolumn(5) eq "g" ? $4 : 1/0) with points pointsize 0.4 title "orienation"

unset multiplot

# plot "./data/robot0.dat" using 1:2 with linespoints title "Robot 1" lw 2 pt 7, \
#     "./data/robot1.dat" using 1:2 with linespoints title "Robot 2" lw 2 pt 9, \
#     "./data/robot2.dat" using 1:2 with linespoints title "Robot 3" lw 2 pt 11, \
#     "./data/robot3.dat" using 1:2 with linespoints title "Robot 4" lw 2 pt 5, \
#     "./data/robot4.dat" using 1:2 with linespoints title "Robot 5" lw 2 pt 13

pause -1
