#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID steering_pid;
  PID velocity_pid;
  double steering_gain = 5.0;
  steering_pid.Init(0.097221, 0.000020, 5.0);
  velocity_pid.Init(0.25, 0., 0.);

  h.onMessage([&steering_pid, &velocity_pid, &steering_gain](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                             uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double steer_value;

          // coefficients to control speed based on current steering target
          float min_target = 40.0;
          float adaptive_target = 60.0;

          // Calculate Steering Value
          steering_pid.UpdateError(cte);
          steer_value = steering_pid.Output();

          // Set target speed
          double target_speed = adaptive_target * (1.0 - 3.5 * fabs(steer_value)) + min_target;
          // calculate speed analogue to cte
          double speed_error = speed - target_speed;

          // Calculate speed value
          velocity_pid.UpdateError(speed_error);
          double speed_value = velocity_pid.Output();

          printf("\nFor Steering PID coefficients are: (%.06f, %.06f, %.06f)", steering_pid.Kp, steering_pid.Ki, steering_pid.Kd);
          printf("\nFor Velocity PID coefficients are: (%.06f, %.06f, %.06f)\n", velocity_pid.Kp, velocity_pid.Ki, velocity_pid.Kd);

          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Velocity Value:" << speed_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}