# Set plot to save to pdf output
set terminal pdfcairo enhanced font 'Helvetica,12'

set fit quiet
set fit logfile '/dev/null'

data_folder = dataset_directory
plots_folder = plots_directory

set style fill empty border -1
set ylabel "Error"
set grid

# Gaussian Function
set samples 1000	# Ensure the Gaussian looks smooth when plotted

# f(x) = 1.0 / (b * sqrt(2 * pi)) * exp(-(x - a)**2 / (2 * b**2))
gaussian(x) = 1.0 / (sigma * sqrt(2 * pi)) * exp(-(x - mu)**2 / (2 * sigma**2))

# set print dataset_directory . "/Robot-Fit-Error-Statistics.dat" 
# print sprintf("Robot ID\t Forward Velocity Mean\t Forward Velocity Variance\t Angular Velocity Mean\t Angular Velocity Variance\t Range Mean\t Range Variance\t Bearing Mean\t Bearing Variance\n")

# Create a plot for every robot
do for [i=1:5] {
	########################
	#Forward-Velocity-Error
	########################

	set title sprintf("Robot %d Forward Velocity Error PDF", i)
	set output sprintf(plots_folder . "/Forward-Velocity/Robot-%d-Foward-Velocity-Error-PDF.pdf" , i)

	# Read Forward Velcoity Error Mean  
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $2 : 1/0) nooutput
	mu = STATS_mean

	# Read Forward Velocity Error Variance
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $3 : 1/0) nooutput
	variance = STATS_mean

	# Calculate the Standard Deviation 
	sigma = sqrt(variance)

	# Apply non-linear regression
	# a = mu
	# b = sigma
	# fit f(x) data_folder . "/Forward-Velocity-Error-PDF.dat" index (i-1) using 1:3 via a, b 
	# line = sprintf("%d\t%f\t%f",i,a,b)

	# Plot the extracted values alongside the fitted Gaussian
	plot \
		data_folder . "/Forward-Velocity-Error-PDF.dat" index (i-1) using 1:3:2 with boxes title "Forward Velocity Error",\
		gaussian(x) title sprintf("Gaussian: μ = %f, σ = %f", mu, sigma) with lines linewidth 2 linecolor rgb "red" #,\
		# f(x) title sprintf("GNU Gaussian: μ = %f, σ = %f", a, b) with lines linewidth 2 linecolor rgb "blue"

	########################
	# Angular-Velocity-Error
	########################

	set title sprintf("Robot %d Angular Velocity Error PDF", i)
	set output sprintf(plots_folder . "/Angular-Velocity/Robot-%d-Angular-Velocity-Error-PDF.pdf" , i)

	# Read Forward Velcoity Error Mean  
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $4 : 1/0) nooutput
	mu = STATS_mean

	# Read Forward Velocity Error Variance
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $5 : 1/0) nooutput
	variance = STATS_mean

	# Calculate the Standard Deviation 
	sigma = sqrt(variance)

	# Apply non-linear regression
	# a = mu
	# b = sigma
	# fit f(x) data_folder . "/Angular-Velocity-Error-PDF.dat" index (i-1) using 1:3 via a, b 
	# line = line . sprintf("\t%f\t%f",a,b)

	# Plot the extracted values alongside the fitted Gaussian
	plot \
		data_folder . "/Angular-Velocity-Error-PDF.dat" index (i-1) using 1:3:2 with boxes title "Angular Velocity Error",\
		gaussian(x) title sprintf("Gaussian: μ = %f, σ = %f", mu, sigma) with lines linewidth 2 linecolor rgb "red" #,\
	# f(x) title sprintf("GNU Gaussian: μ = %f, σ = %f", a, b) with lines linewidth 2 linecolor rgb "blue"

	########################
	# Range Error
	########################
	set title sprintf("Robot %d Range Error PDF", i)
	set output sprintf(plots_folder . "/Range/Robot-%d-Range-Error-PDF.pdf" , i)

	# Read Forward Velcoity Error Mean  
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $6 : 1/0) nooutput
	mu = STATS_mean

	# Read Forward Velocity Error Variance
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $7 : 1/0) nooutput
	variance = STATS_mean

	# Calculate the Standard Deviation 
	sigma = sqrt(variance)

	# Apply non-linear regression
	# a = mu
	# b = sigma
	# fit f(x) data_folder . "/Range-Error-PDF.dat" index (i-1) using 1:3 via a, b
	# line = line . sprintf("\t%f\t%f",a,b)

	# Plot the extracted values alongside the fitted Gaussian
	plot \
		data_folder . "/Range-Error-PDF.dat" index (i-1) using 1:3:2 with boxes title "Range Error",\
		gaussian(x) title sprintf("Gaussian: μ = %f, σ = %f", mu, sigma) with lines linewidth 2 linecolor rgb "red" #,\
		# f(x) title sprintf("GNU Gaussian: μ = %f, σ = %f", a, b) with lines linewidth 2 linecolor rgb "blue"

	########################
	# Bearing-Error
	########################

	set title sprintf("Robot %d Bearing Error PDF", i)
	set output sprintf(plots_folder . "/Bearing/Robot-%d-Bearing-Error-PDF.pdf" , i)

	# Read Forward Velcoity Error Mean
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $8 : 1/0) nooutput
	mu = STATS_mean

	# Read Forward Velocity Error Variance
	stats data_folder . "/Robot-Error-Statistics.dat" using ($1==i ? $9 : 1/0) nooutput
	variance = STATS_mean

	# Calculate the Standard Deviation 
	sigma = sqrt(variance)

	# Apply non-linear regression
	# a = mu
	# b = sigma
	# fit f(x) data_folder . "/Bearing-Error-PDF.dat" index (i-1) using 1:3 via a, b 
	# line = line . sprintf("\t%f\t%f",a,b)
	# print line

	# Plot the extracted values alongside the fitted Gaussian
	plot \
		data_folder . "/Bearing-Error-PDF.dat" index (i-1) using 1:3:2 with boxes title "Bearing Error",\
		gaussian(x) title sprintf("Gaussian: μ = %f, σ = %f", mu, sigma) with lines linewidth 2 linecolor rgb "red" #,\
		# f(x) title sprintf("GNU Gaussian: μ = %f, σ = %f", a, b) with lines linewidth 2 linecolor rgb "blue"

}
