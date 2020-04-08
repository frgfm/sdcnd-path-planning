#include "planner.h"

// Constructor
Planner::Planner(const double &max_front, const double &max_rear,
                 const float &lwidth)
    : module(lwidth) {
  front_margin = max_front;
  rear_margin = max_rear;
  Planner::reset_env();
}

// Destructor
Planner::~Planner() = default;

void Planner::reset_env() {
  lane_avails = {true, true, true};
  lane_transitions = {true, true, true};
}

void Planner::sense(const vector<vector<double> > &sensor_fusion,
                    const double &delta_t, const double &car_s) {
  Planner::reset_env();
  module.update(sensor_fusion, delta_t, car_s);
  for (uint i = 0; i < lane_avails.size(); ++i) {
    lane_avails[i] = (module.front_margins[i] > front_margin) &&
                     (module.rear_margins[i] > rear_margin);
    // Allow transition through lanes with lower front margin
    lane_transitions[i] = (module.front_margins[i] > (front_margin / 2)) &&
                          (module.rear_margins[i] > rear_margin);
  }
}

void Planner::update(int &lane, double &target_vel) {
  // Planning (lane selection and velocity update)
  if (!lane_avails[lane]) {
    // Check is lane change is possible
    if (((lane > 0) && lane_avails[lane - 1]) ||
        ((lane < 2) && lane_avails[lane + 1])) {
      // Lane selection
      uint best_lane = lane;
      double max_margin = front_margin;

      // Take best front margin
      for (int i = 0; i < lane_avails.size(); ++i) {
        if (i == lane) {
          continue;
        }
        // Good candidate
        if (lane_avails[i] && (module.front_margins[i] > max_margin)) {
          // Deal with edge case of 2-lanes difference
          if (std::abs(lane - i) == 2) {
            // Get inbetween lane
            uint lane_ = lane + 1;
            if (i < lane) {
              lane_ = lane - 1;
            }
            // Check if that lane is available
            if (!lane_transitions[lane_]) {
              continue;
            } else {
              // Take intermediate lane for now (the planner will select the next best lane afterwards)
              best_lane = lane_;
            }
          } else {
            best_lane = i;
          }
          max_margin = module.front_margins[i];
        }
      }

      lane = best_lane;

      // Adapt motion
    } else {
      // No lane change available --> shadow front vehicle velocity
      target_vel = module.front_speeds[lane];
    }
  }
}
