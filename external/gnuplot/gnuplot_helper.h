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

  unsigned short x = 1;
  unsigned short y = 2;

  PlotStyle style = POINTS;

  PointType pointtype = CIRCLE;
  double pointsize = 1.0;

  Colour linecolor = NONE;
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
      << "with " << to_string(settings.style) << " pointtype "
      << to_string(settings.pointtype) << " pointsize " << settings.pointsize;

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
  std::string terminal_size = " size 1336,768 ";

  std::string terminal_settings = "";
  terminal_settings += " set mouse\n";
  terminal_settings += " set term " + terminal_type +
                       std::to_string(terminal_number) + terminal_size +
                       " noraise\n";
  terminal_settings += "set samples 1000\n";

  return terminal_settings;
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

inline std::string setGrid() { return "set grid\n"; }
inline std::string unsetGrid() { return "unset grid\n"; }
} // namespace gnuplot
