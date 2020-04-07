#ifndef PLANNER_H
#define PLANNER_H

#include "spdlog/spdlog.h"

using std::vector;

void plan_motion(uint &lane, double &target_vel, float &spline_dist_,
                 const std::array<bool, 3> &lane_avails,
                 const std::array<bool, 3> &lane_transitions,
                 const std::array<double, 3> &front_margins,
                 const std::array<double, 3> &front_speeds,
                 const float &spline_dist, const double &front_margin) {
  // Planning (lane selection and velocity update)
  if (!lane_avails[lane]) {
    spdlog::info("Lane availability: {} | {} | {}", lane_avails[0],
                 lane_avails[1], lane_avails[2]);
    // Check is lane change is possible
    if (((lane > 0) && lane_avails[lane - 1]) ||
        ((lane < 2) && lane_avails[lane + 1])) {
      // Lane selection
      uint best_lane = lane;
      double max_margin = front_margin;

      // Take best front margin
      for (uint i = 0; i < lane_avails.size(); i++) {
        if (i == lane) {
          continue;
        }
        // Good candidate
        if (lane_avails[i] && (front_margins[i] > max_margin)) {
          // Deal with edge case of 2-lanes difference
          if (fabs(lane - i) == 2) {
            // Get inbetween lane
            uint lane_ = lane + 1;
            if (i < lane) {
              lane_ = lane - 1;
            }
            // Check if that lane is available
            if (!lane_transitions[lane_]) {
              continue;
              // Anticipate jerk issue (2 lane difference with same
              // interpolation isnt smooth)
            } else {
              // Limit jerk by changing spline interpolation
              spline_dist_ = 2 * spline_dist;
            }
          } else {
            spline_dist_ = spline_dist;
          }
          best_lane = i;
          max_margin = front_margins[i];
        }
      }

      lane = best_lane;

      // Adapt motion
    } else {
      // No lane change available --> shadow front vehicle velocity
      target_vel = front_speeds[lane];
    }
  }
}

#endif  // PLANNER_H
