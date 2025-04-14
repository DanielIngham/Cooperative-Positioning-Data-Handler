set xlabel "Time [s]" 
set xrange [830:910]

#set ylabel "Subjects"
#plot \
#"../data/MRCLAM_Dataset1/output/Measurement.dat" index 0 using 1:2 with points title "Measured",\
#"../data/MRCLAM_Dataset1/output/Groundtruth-Measurement.dat" index 0 using (stringcolumn(3) eq 'r' ? $1 : 1/0):(stringcolumn(3) eq 'r' ? $2 : 1/0) with points title "Robot", \
#"" index 0 using (stringcolumn(3) eq 'l' ? $1 : 1/0):(stringcolumn(3) eq 'l' ? $2 : 1/0) with points title "Landmark"
#pause -1

set ylabel "Range [m]"
plot \
for [i=1:4] "../data/MRCLAM_Dataset5/output/Relative_robot.dat" index 0 using ($2 == i ? $1 : 1/0):($2 == i ? $3 : 1/0) with points ps 0.4 title sprintf("Robot %d", i), \
"../data/MRCLAM_Dataset5/output/Measurement.dat" index 0 using 1:3 with points title "Measured",\
"../data/MRCLAM_Dataset5/output/Groundtruth-Measurement.dat" index 0 using (stringcolumn(3) eq 'r' ? $1 : 1/0):(stringcolumn(3) eq 'r' ? $4 : 1/0) with points title "Robot", \
"" index 0 using (stringcolumn(3) eq 'l' ? $1 : 1/0):(stringcolumn(3) eq 'l' ? $4 : 1/0) with points title "Landmark"

pause -1
#for [i=6:20] "../data/MRCLAM_Dataset5/output/Relative_landmark.dat" index 0 using ($2 == i ? $1 : 1/0):($2 == i ? $3 : 1/0) with points ps 0.4 title sprintf("Landmark %d", i), \

set ylabel "Bearing [rad]"
plot \
"../data/MRCLAM_Dataset1/output/Measurement.dat" index 0 using 1:4 with points title "Measured",\
"../data/MRCLAM_Dataset1/output/Groundtruth-Measurement.dat" index 0 using (stringcolumn(3) eq 'r' ? $1 : 1/0):(stringcolumn(3) eq 'r' ? $5 : 1/0) with points title "Robot", \
"" index 0 using (stringcolumn(3) eq 'l' ? $1 : 1/0):(stringcolumn(3) eq 'l' ? $5 : 1/0) with points title "Landmark"

pause -1
