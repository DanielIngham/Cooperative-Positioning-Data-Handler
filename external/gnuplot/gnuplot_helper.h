/**
 * @file gnuplot_helper.h
 * @brief Functions that allow for easier use of the gnuplot iostream header.
 * @author Daniel Ingham
 * @date 2025-08-08
 */
#include <sstream>
#include <string>

namespace gnuplot {

/**
 * Most plot styles available to gnuplot (i.e. scatter plot (points),
 * histograms, etc).
 */
enum PlotStyle {
  POINTS,
  LINES,
  LINESPOINTS,
  DOTS,
  IMPULSES,
  STEPS,
  FSTEPS,
  HISTOGRAMS,
  BOXES,
  ERRORBARS,
  XERRORBARS,
  YERRORBARS
};

/**
 * Types of symbols available to represent points in gnuplot.
 */
enum PointType {
  DOT,
  PLUS,
  CROSS,
  STAR,
  BOX,
  FILLED_BOX,
  CIRCLE,
  FILLED_CIRCLE,
  TRIANGLE_UP,
  FILLED_TRIANGLE_UP,
  TRIANGLE_DOWN,
  FILLED_TRIANGLE_DOWN,
  DIAMOND,
  FILLED_DIAMOND,
  PENTAGON,
  FILLED_PENTAGON,
};

/**
 * Most plot colours available in gnuplot.
 */
enum Colour {
  NONE,
  BLACK,
  WHITE,
  RED,
  GREEN,
  BLUE,
  CYAN,
  MAGENTA,
  YELLOW,
  DARK_RED,
  DARK_GREEN,
  DARK_BLUE,
  LIGHT_RED,
  LIGHT_GREEN,
  LIGHT_BLUE,
  ALICEBLUE,
  ANTIQUEWHITE,
  AQUA,
  AZURE,
  BEIGE,
  BISQUE,
};

/**
 * Data structure that houses the settings required by gnuplot for plotting.
 * @struct PlotSettings
 */
struct PlotSettings {
  std::string title = "";

  /** Element in the tuple that correponds to the x axis data point. */
  unsigned short x = 1;

  /** Element in the tuple that correponds to the y axis data point. */
  unsigned short y = 2;

  /* The type of plot. */
  PlotStyle style = POINTS;

  /* The style of the symbols that represent the points in the point plot. */
  PointType pointtype = CIRCLE;
  /* Size of the points. */
  double pointsize = 1.0;

  /* Colour of the plot line. Note that NONE means gnuplot automatically assigns
   * it. */
  Colour linecolor = NONE;

  /* Width of the plot line. */
  double linewidth = 1.0;
};

/**
 * Converts the PointStyle enum into a string to parse to gnuplot.
 * @param style PlotStyle element.
 * @return string representing the plot style that gnuplot can use.
 */
inline std::string to_string(PlotStyle style) {
  switch (style) {
  case POINTS:
    return "points";
  case LINES:
    return "lines";
  case LINESPOINTS:
    return "linespoints";
  case DOTS:
    return "dots";
  case IMPULSES:
    return "impulses";
  case STEPS:
    return "steps";
  case FSTEPS:
    return "fsteps";
  case HISTOGRAMS:
    return "histograms";
  case BOXES:
    return "boxes";
  case ERRORBARS:
    return "errorbars";
  case XERRORBARS:
    return "xerrorbars";
  case YERRORBARS:
    return "yerrorbars";

  default:
    return "points";
  }
}

/**
 * Converts the Colour enum into a string to parse to gnuplot.
 * @param colour Colour element.
 * @return string representing the plot colour that gnuplot can use.
 */
inline std::string to_string(Colour colour) {
  switch (colour) {
  case NONE:
    return "";
  case BLACK:
    return "black";
  case WHITE:
    return "white";
  case RED:
    return "red";
  case GREEN:
    return "green";
  case BLUE:
    return "blue";
  case CYAN:
    return "cyan";
  case MAGENTA:
    return "magenta";
  case YELLOW:
    return "yellow";
  case DARK_RED:
    return "dark-red";
  case DARK_GREEN:
    return "dark-green";
  case DARK_BLUE:
    return "dark-blue";
  case LIGHT_RED:
    return "light-red";
  case LIGHT_GREEN:
    return "light-green";
  case LIGHT_BLUE:
    return "light-blue";
  case ALICEBLUE:
    return "aliceblue";
  case ANTIQUEWHITE:
    return "antiquewhite";
  case AQUA:
    return "aqua";
  case AZURE:
    return "azure";
  case BEIGE:
    return "beige";
  case BISQUE:
    return "bisque";
  default:
    return "";
  }
}

/**
 * Converts a PointType element to a string the gnuplot can process.
 * @param type symbol that should be drawn representing a data point.
 * @returns string for gnuplot command.
 */
inline std::string to_string(PointType type) {
  return std::to_string(static_cast<int>(type));
}

/**
 * Creates a gnuplot command based off the fields in a PlotSettings instance.
 * @param settings settings for the plot.
 * @returns string representing the gnuplot command.
 */
inline std::string command(const PlotSettings &settings) {
  std::ostringstream oss;

  oss << " using " << settings.x << ":" << settings.y << " "
      << "title \"" << settings.title << "\" "
      << "with " << to_string(settings.style);

  if (settings.style == POINTS || settings.style == LINESPOINTS) {
    oss << " pointtype " << to_string(settings.pointtype) << " pointsize "
        << settings.pointsize;
  }

  if (settings.linecolor != NONE) {
    oss << " linecolor rgb \"" << to_string(settings.linecolor) << "\" ";
  }

  oss << " linewidth " << settings.linewidth;

  return oss.str();
}

/**
 * Sets the settings regarding gnuplot terminal output.
 * @note At the moment this is just for the qt terminal and all settings are
 * hardcoded.
 * @param terminal_number the number of the terminal instance.
 */
inline std::string setTerminal(unsigned short terminal_number) {
  /* TODO: add more terminal settings functionality. */
  std::string terminal_type = " qt ";
  std::string terminal_size = "  1336,768 ";

  size_t total_samples = 1000U;

  std::ostringstream oss;

  /* Allow for mouse interaction. */
  oss << " set mouse\n";

  /* Set the number of samples that should represent a plot. */
  oss << "set samples " << total_samples << "\n";

  /* Set the terminal type and instance number. */
  oss << " set term " << terminal_type << terminal_number;

  /* Set the terminal size. */
  oss << " size " << terminal_size;

  /* When replotting or updating the graph, do not automatically bring the plot
   * window to the front (raise it above other windows)*/
  oss << " noraise\n";

  oss << '\n';

  return oss.str();
}

/**
 * Interface that creates a string that represents the gnuplot command required
 * to create a multiplot in gnuplot.
 * @param rows The number of rows in the multiplot.
 * @param columns The number of columns in the multiplot.
 * @param title The title of the multiplot.
 */
inline std::string setMultiplot(unsigned short rows, unsigned short columns,
                                const std::string title = "") {
  std::ostringstream oss;

  oss << "set multiplot ";

  /* Adding layout constraints */
  oss << "layout " << rows << "," << columns << " ";

  /* Adding title */
  oss << "title \"" + title + '"';

  oss << '\n';

  return oss.str();
}

/*
 * Interface that creates a string that represents the gnuplot command required
 * to unset a multiplot.
 */
inline std::string unsetMultiplot() { return "unset multiplot\n"; }

/**
 * Creates string that gnuplot uses to determine whether it should plot a grid
 * or not.
 * @param on Turn on the grid.
 */
inline std::string grid(const bool on = true) {
  return on ? "set grid\n" : "unset grid\n";
}

} // namespace gnuplot
