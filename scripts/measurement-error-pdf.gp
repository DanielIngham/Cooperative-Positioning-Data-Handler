set title "Forward Velocity Error"
set ylabel "Error [m/s]"
set style fill empty border -1

plot \
"../data/MRCLAM_Dataset1/output/Forward-Velocity-Error-PDF.dat" using ($4 == 1 ? $1 : 1/0):($4 == 1 ? $3 : 1/0):($4 == 1 ? $2 : 1/0) with boxes title "Robot 1", \
""  using ($4 == 2 ? $1 : 1/0):($4 == 2 ? $3 : 1/0):($4 == 2 ? $2 : 1/0) with boxes title "Robot 2", \
""  using ($4 == 3 ? $1 : 1/0):($4 == 3 ? $3 : 1/0):($4 == 3 ? $2 : 1/0) with boxes title "Robot 3", \
""  using ($4 == 4 ? $1 : 1/0):($4 == 4 ? $3 : 1/0):($4 == 4 ? $2 : 1/0) with boxes title "Robot 4", \
""  using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title "Robot 5"

pause -1


plot \
"../data/MRCLAM_Dataset1/output/Angular-Velocity-Error-PDF.dat" using ($4 == 1 ? $1 : 1/0):($4 == 1 ? $3 : 1/0):($4 == 1 ? $2 : 1/0) with boxes title "Robot 1", \
""  using ($4 == 2 ? $1 : 1/0):($4 == 2 ? $3 : 1/0):($4 == 2 ? $2 : 1/0) with boxes title "Robot 2", \
""  using ($4 == 3 ? $1 : 1/0):($4 == 3 ? $3 : 1/0):($4 == 3 ? $2 : 1/0) with boxes title "Robot 3", \
""  using ($4 == 4 ? $1 : 1/0):($4 == 4 ? $3 : 1/0):($4 == 4 ? $2 : 1/0) with boxes title "Robot 4", \
""  using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title "Robot 5"

pause -1
