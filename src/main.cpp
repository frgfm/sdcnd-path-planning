#include <uWS/uWS.h>
#include <string>
#include <vector>
#include "controller.h"
#include "helpers.h"
#include "json.hpp"
#include "perception.h"
#include "planner.h"
#include "spdlog/spdlog.h"

// for convenience
using nlohmann::json;
using std::exception;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  // x, y, s, dx, dy
  std::array<vector<double>, 5> map_waypoints;

  // Map data safeguard
  struct PPException : public exception {
    const char *what() const throw() {
      return "Unable to access highway map file!";
    }
  };

  if (!helpers.read_map_data("../data/highway_map.csv", map_waypoints)) {
    spdlog::error("Unable to access highway map file!");
    throw PPException();
  }
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Lanes are numbered (0 | 1 | 2)
  // Start on lane 1 (middle lane)
  uint lane = 1;

  // Inicial velocity, and also reference velocity to target.
  double current_vel = 0.0;           // mph
  const float spline_dist = 30;       // m
  const double target_vel = 49.7;     // mph
  const double vel_delta = 3 * .224;  // 5m/s
  const double refresh = .02;      // second
  const float lane_width = 4;      // m
  const double front_margin = 30;  // m
  const double rear_margin = 5;    // m

  Controller controller(vel_delta, lane_width, refresh, map_waypoints);

  // True when the ego-car is changing lane.
  bool is_changing_lane = false;
  double end_change_lane_s = 0.0;

  h.onMessage([&map_waypoints, &lane, &current_vel, &vel_delta, &target_vel,
               &refresh, &lane_width, &spline_dist, &front_margin, &rear_margin,
               &controller](uWS::WebSocket<uWS::SERVER> ws, char *data,
                            size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = helpers.hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          // Avoids collision
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // Perception
          std::array<double, 3> front_margins = {
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity()};
          std::array<double, 3> rear_margins = {
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity()};
          std::array<double, 3> front_speeds;
          std::array<double, 3> rear_speeds;

          // Use sensor fusion to update perception
          update_perception(front_margins, rear_margins, front_speeds,
                            rear_speeds, sensor_fusion,
                            static_cast<double>(prev_size) * refresh,
                            lane_width, car_s);

          // Define lane availability
          std::array<bool, 3> lane_avails = {true, true, true};
          std::array<bool, 3> lane_transitions = {true, true, true};
          for (uint i = 0; i < lane_avails.size(); i++) {
            lane_avails[i] = (front_margins[i] > front_margin) &&
                             (rear_margins[i] > rear_margin);
            // Allow transition through lanes with lower front margin
            lane_transitions[i] = (front_margins[i] > (front_margin / 2)) &&
                                  (rear_margins[i] > rear_margin);
          }

          float spline_dist_ = spline_dist;
          // bool slow_down = false;
          double target_vel_ = target_vel;

          // Perform lane selection and velocity update
          plan_motion(lane, target_vel_, spline_dist_, lane_avails,
                      lane_transitions, front_margins, front_speeds,
                      spline_dist, front_margin);

          // Let controller update its information
          controller.update_readings(car_x, car_y, car_yaw, current_vel, car_s,
                                     previous_path_x, previous_path_y);
          // Set the new target speed
          current_vel = controller.update_velocity(target_vel_);
          // Compute the trajectory
          std::array<vector<double>, 2> next_coords =
              controller.get_trajectory(lane, spline_dist_);

          json msgJson;

          msgJson["next_x"] = next_coords[0];
          msgJson["next_y"] = next_coords[1];

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection(
      [&h, &current_vel](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        spdlog::info("Environment session connected!");
        // Ensure that new driving sessions starts with zero velocity
        current_vel = 0.0;
      });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    spdlog::info("Disconnected from session");
  });

  int port = 4567;
  if (h.listen(port)) {
    spdlog::info("Listening to port {}", port);
  } else {
    spdlog::error("Failed to listen to port {}", port);
    return -1;
  }

  h.run();
}
