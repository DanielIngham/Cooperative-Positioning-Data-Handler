#include <sstream>
#include <string>

namespace gnuplot {

// gnuplot_ << "using 1:2 with points pointsize "1.0 pointtype 6 title
// 'Interpolated',";
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

inline std::string to_string(PointType type) {
  return std::to_string(static_cast<int>(type));
}

inline std::string command(const PlotSettings &s) {
  std::ostringstream oss;

  oss << " using " << s.x << ":" << s.y << " "
      << "title \"" << s.title << "\" "
      << "with " << to_string(s.style) << " pointtype "
      << to_string(s.pointtype) << " pointsize " << s.pointsize;

  if (s.linecolor != NONE) {
    oss << " linecolor rgb \"" << to_string(s.linecolor) << "\" ";
  }

  oss << " linewidth " << s.linewidth;

  return oss.str();
}

inline std::string setTerminal(unsigned short terminal_number) {
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

inline std::string setMultiplot(unsigned short rows, unsigned short columns,
                                const std::string title = "") {

  std::string multi_plot_settings = "set multiplot ";

  /* Adding layout constraints */
  multi_plot_settings +=
      "layout " + std::to_string(rows) + "," + std::to_string(columns) + " ";

  if (title != "") {
    multi_plot_settings += "title \"" + title + '"';
  }

  multi_plot_settings += '\n';

  return multi_plot_settings;
}

inline std::string unsetMultiplot() { return "unset multiplot\n"; }

inline std::string setGrid() { return "set grid\n"; }
inline std::string unsetGrid() { return "unset grid\n"; }
} // namespace gnuplot
