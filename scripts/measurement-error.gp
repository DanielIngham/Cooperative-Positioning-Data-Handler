set multiplot layout 2,1 title "Measurement Error"
set xlabel "Time [s]"
set grid 
set key inside
plot for [i=1:9] sprintf("../data/MRCLAM_Dataset%d/output/Measurement-Error.dat", i) index 0 using 1:3 with points title sprintf("Dataset %d", i)
plot for [i=1:9] sprintf("../data/MRCLAM_Dataset%d/output/Measurement-Error.dat", i) index 0 using 1:4 with linespoints title sprintf("Dataset %d", i)

unset multiplot
pause -1
