/**
 * @file Plotter.h
 * @brief Contains gnuplot functionality.
 * @date 2025-08-06
 */
#pragma once

#include "DataHandler.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

/* Warn about depreciated functions. */
#define GNUPLOT_DEPRECATE_WARN
#include "./external/gnuplot/gnuplot-iostream.h"

#include "DataHandler.h"

namespace Data {

namespace Plot {

inline void demo_animation() {

  Gnuplot gp;

  std::cout << "Press Ctrl-C to quit (closing gnuplot window doesn't quit)."
            << std::endl;

  gp << "set yrange [-1:1]\n";

  const int N = 1000;
  std::vector<double> pts(N);

  double theta = 0;
  while (1) {
    for (int i = 0; i < N; i++) {
      double alpha = (static_cast<double>(i) / N - 0.5) * 10;
      pts[i] = sin(alpha * 8.0 + theta) * exp(-alpha * alpha / 2.0);
    }

    gp << "plot '-' binary" << gp.binFmt1d(pts, "array")
       << "with lines notitle\n";
    gp.sendBinary1d(pts);
    gp.flush();

    theta += 0.2;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

inline void plotGroundruth(Handler &handler) {

  Gnuplot gp;
  std::vector<Robot> &robots = handler.getRobots();
  std::vector<Landmark> &landmarks;

  std::vector<double> x;
  std::vector<double> y;

  for (size_t i = 0; i < robots[0].groundtruth.states.size(); ++i) {
    x.push_back(robots[0].groundtruth.states[i].x);
    y.push_back(robots[0].groundtruth.states[i].y);
  }

  // Plot using std::pair (XY format)
  std::vector<std::pair<double, double>> xy;
  for (size_t i = 0; i < x.size(); ++i) {
    xy.emplace_back(x[i], y[i]);
  }
  gp << "set term wxt noraise\n";
  gp << "plot '-' with lines title 'xy plot'\n";
  gp.send1d(xy);

  // gp << "plot '-' binary" << gp.binFmt1d(pts, "array")
  //    << "with lines notitle\n";
  // gp.sendBinary1d(pts);

  gp.flush();
}
} // namespace Plot

} // namespace Data
