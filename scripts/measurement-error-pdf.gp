set title "Range Error PDF"
set style fill empty border -1

set ylabel "Error [m/s]"

# Define the Gaussian function
set samples 1000	# Ensure the Gaussian looks smooth when plotted
gaussian(x) = 1.0 / (sigma * sqrt(2 * pi)) * exp(-(x - mu)**2 / (2 * sigma**2))

set terminal pdfcairo enhanced font 'Helvetica,12'
do for [i=1:5] {
	set output sprintf("Robot-%d-Range-Error.pdf" , i)

	# Read Forward Velcoity Error Mean  
	#stats "../data/MRCLAM_Dataset1/output/Robot-Error-Statistics.dat" index (i-1) using 2 nooutput
	stats "../data/MRCLAM_Dataset1/output/Robot-Error-Statistics.dat" using ($1==(i-1) ? $6 : 1/0) nooutput
	mu = STATS_mean

	# Read Forward Velocity Error Variance
	# stats "../data/MRCLAM_Dataset1/output/Robot-Error-Statistics.dat" index (i-1) using 3 nooutput
	stats "../data/MRCLAM_Dataset1/output/Robot-Error-Statistics.dat" using ($1==(i-1) ? $7 : 1/0) nooutput

	# Calculate the Standard Deviation 
	sigma = sqrt(variance)
	print sigma

	# Plot the extracted values alongside the fitted Gaussian
	plot "../data/MRCLAM_Dataset1/output/Range-Error-PDF.dat" using ($4 == i ? $1 : 1/0):($4 == i ? $3 : 1/0):($4 == i ? $2 : 1/0) with boxes title "Error",\
	gaussian(x) title sprintf("Gaussian: μ = %f, σ = %f", mu, sigma) with lines linewidth 2 linecolor rgb "red"
}

#plot \
#"../data/MRCLAM_Dataset1/output/Range-Error-PDF.dat" using ($4 == 1 ? $1 : 1/0):($4 == 1 ? $3 : 1/0):($4 == 1 ? $2 : 1/0) with boxes title "Robot 1", \
#"../data/MRCLAM_Dataset1/output/Range-Error-PDF.dat" using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title "Robot 5", \
#"../data/MRCLAM_Dataset1/output/Range-Error-PDF.dat" using ($4 == 4 ? $1 : 1/0):($4 == 4 ? $3 : 1/0):($4 == 4 ? $2 : 1/0) with boxes title "Robot 4", \
#"../data/MRCLAM_Dataset1/output/Range-Error-PDF.dat" using ($4 == 3 ? $1 : 1/0):($4 == 3 ? $3 : 1/0):($4 == 3 ? $2 : 1/0) with boxes title "Robot 3", \
#"../data/MRCLAM_Dataset1/output/Range-Error-PDF.dat" using ($4 == 2 ? $1 : 1/0):($4 == 2 ? $3 : 1/0):($4 == 2 ? $2 : 1/0) with boxes title "Robot 2", \
#""  using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title "Robot 5"

#for [i=2:9] sprintf("../data/MRCLAM_Dataset%d/output/Range-Error-PDF.dat", i)  using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title sprintf("R 5, D %d", i)

#pause -1

#set title "Bearing Error PDF"
#set xrange[-0.2:0.2]
#plot \
#for [i=1:9] sprintf("../data/MRCLAM_Dataset%d/output/Bearing-Error-PDF.dat", i)  using ($4 == 1 ? $1 : 1/0):($4 == 1 ? $3 : 1/0):($4 == 1 ? $2 : 1/0) with boxes title sprintf("R 5, D %d", i)
#"../data/MRCLAM_Dataset1/output/Bearing-Error-PDF.dat" using ($4 == 5 ? $1 : 1/0):($4 == 5 ? $3 : 1/0):($4 == 5 ? $2 : 1/0) with boxes title "Robot 5"
#"../data/MRCLAM_Dataset1/output/Bearing-Error-PDF.dat" using ($4 == 4 ? $1 : 1/0):($4 == 4 ? $3 : 1/0):($4 == 4 ? $2 : 1/0) with boxes title "Robot 4", \
#"../data/MRCLAM_Dataset1/output/Bearing-Error-PDF.dat" using ($4 == 3 ? $1 : 1/0):($4 == 3 ? $3 : 1/0):($4 == 3 ? $2 : 1/0) with boxes title "Robot 3", \
#"../data/MRCLAM_Dataset1/output/Bearing-Error-PDF.dat" using ($4 == 2 ? $1 : 1/0):($4 == 2 ? $3 : 1/0):($4 == 2 ? $2 : 1/0) with boxes title "Robot 2", \
#"../data/MRCLAM_Dataset1/output/Bearing-Error-PDF.dat" using ($4 == 1 ? $1 : 1/0):($4 == 1 ? $3 : 1/0):($4 == 1 ? $2 : 1/0) with boxes title "Robot 1"

#pause -1
