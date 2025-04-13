set title "Forward Velocity Error"
set ylabel "Error [m/s]"
set style fill empty border -1
#plot "./data/robot4-Forward-Error-PDF.dat" using 1:3:2 with boxes notitle
#plot "./data/robot4-Angular-Error-PDF.dat" using 1:3:2 with boxes notitle

# plot "./data/robot0-Groundtruth.dat" using (stringcolumn(5) eq "r" ? $1 : 1/0):(stringcolumn(5) eq "r" ? $2 : 1/0) with points pointsize 1.4 linecolor rgb "red" pointtype 7 title "Raw",\
plot \
"../data/MRCLAM_Dataset1/output/Forward-Velocity-Error-PDF.dat" using ($4 == 1 ? $1 : 1/0):($4 == 1 ? $3 : 1/0):($4 == 1 ? $2 : 1/0) with boxes title "Robot 1", \
"../data/MRCLAM_Dataset1/output/Forward-Velocity-Error-PDF.dat" using ($4 == 2 ? $1 : 1/0):($4 == 2 ? $3 : 1/0):($4 == 2 ? $2 : 1/0) with boxes title "Robot 2", \
"../data/MRCLAM_Dataset1/output/Forward-Velocity-Error-PDF.dat" using ($4 == 3 ? $1 : 1/0):($4 == 3 ? $3 : 1/0):($4 == 3 ? $2 : 1/0) with boxes title "Robot 3", \
"../data/MRCLAM_Dataset1/output/Forward-Velocity-Error-PDF.dat" using ($4 == 4 ? $1 : 1/0):($4 == 4 ? $3 : 1/0):($4 == 4 ? $2 : 1/0) with boxes title "Robot 4", \
"../data/MRCLAM_Dataset1/output/Forward-Velocity-Error-PDF.dat" using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title "Robot 5"

pause -1


plot \
"./data/robot0-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 1", \
"./data/robot1-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 2", \
"./data/robot2-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 3", \
"./data/robot3-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 4", \
"./data/robot4-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 5"

pause -1
