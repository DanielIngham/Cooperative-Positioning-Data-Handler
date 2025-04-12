set title "Forward Velocity Error"
set ylabel "Error [m/s]"
set style fill empty border -1
#plot "./data/robot4-Forward-Error-PDF.dat" using 1:3:2 with boxes notitle
#plot "./data/robot4-Angular-Error-PDF.dat" using 1:3:2 with boxes notitle

plot "./data/robot0-Forward-Error-PDF.dat" using 1:3:2 with boxes title "Robot 1", \
"./data/robot1-Forward-Error-PDF.dat" using 1:3:2 with boxes title "Robot 2", \
"./data/robot2-Forward-Error-PDF.dat" using 1:3:2 with boxes title "Robot 3", \
"./data/robot3-Forward-Error-PDF.dat" using 1:3:2 with boxes title "Robot 4", \
"./data/robot4-Forward-Error-PDF.dat" using 1:3:2 with boxes title "Robot 5"

pause -1


plot "./data/robot0-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 1", \
"./data/robot1-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 2", \
"./data/robot2-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 3", \
"./data/robot3-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 4", \
"./data/robot4-Angular-Error-PDF.dat" using 1:3:2 with boxes title "Robot 5"

pause -1
