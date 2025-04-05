# Plot settings
set title "Robot Trajectories"
set xlabel "X Position"
set ylabel "Y Position"
set grid
set key outside

plot "./data/robot0.dat" using 1:2 with linespoints title "Robot 1" lw 2 pt 7, \
     "./data/robot1.dat" using 1:2 with linespoints title "Robot 2" lw 2 pt 9, \
     "./data/robot2.dat" using 1:2 with linespoints title "Robot 3" lw 2 pt 11, \
     "./data/robot3.dat" using 1:2 with linespoints title "Robot 4" lw 2 pt 5, \
     "./data/robot4.dat" using 1:2 with linespoints title "Robot 5" lw 2 pt 13

pause -1
