#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/**
 * Restarts the simulator.
 *
 * @param ws websocket of simulator.
 */
void Restart(uWS::WebSocket<uWS::SERVER> ws) {
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

/**
 * Prints the command line parameter usage.
 *
 * @param name Program name.
 */
static void show_usage(std::string name)
{
  cerr << "Usage: " << name << " <option(s)>\n"
  << "Options:\n"
  << "\t-h,--help\t\tShow this help message\n"
  << "\t-o,--output OUTPUT\tSpecify the output file\n"
  << endl;
}

/**
 * Main routine.
 */
int main(int argc, char** argv)
{
  // check arguments
  if (argc > 3) {
    show_usage(argv[0]);
    return 1;
  }
  
  string output;
  ofstream output_file;
  
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    
    if ((arg == "-h") || (arg == "--help")) {
      show_usage(argv[0]);
      return 0;
    } else if ((arg == "-o") || (arg == "--output")) {
      // setup output file
      if (i + 1 < argc) {
        output = argv[++i];
        output_file.open(output, ios_base::out);
        
        if (!output_file.is_open()) {
          cout << "Unable to create file " << output << "." << endl;
          return 1;
        } else {
          cout << "Created output file: " << output << endl;
        }
      } else {
        cerr << "--output option requires one argument." << endl;
        return 1;
      }
    }
  }

  uWS::Hub h;

  PID pid_steer;
  PID pid_speed;
  pid_steer.Init(0.1, 0.00025, 3.5);
  pid_speed.Init(0.5, 0.000051, 0.0);
  double target_speed = 30;
  double prev_time = clock();
  double total_time = 0.0;

  h.onMessage([&pid_steer, &pid_speed, &target_speed, &prev_time, &total_time, &output, &output_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          double time = clock();
          double dt = (time - prev_time) / CLOCKS_PER_SEC;
          total_time += dt;
          prev_time = time;
          
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          
          // update steering PID controller
          //pid_steer.UpdateError(cte, dt);
          pid_steer.UpdateError(cte);
          double steer_value = pid_steer.GetControlValue();
          steer_value = std::min(std::max(steer_value, -1.0), 1.0); //normalize the steer_value in range [-1,1]
          
          // update speed PID controller
          //pid_speed.UpdateError(speed - target_speed, dt);
          pid_speed.UpdateError(speed - target_speed);
          double throttle_value = pid_speed.GetControlValue();
          
          // write file with format: dt, cte, speed, angle, steering value, throttle value
          if (output_file.is_open()) {
            output_file << total_time << "," << dt << "," << cte << "," << speed << "," << angle << "," << steer_value << "," << throttle_value << "\n";
          }

          // DEBUG
          std::cout << "dt: " << dt << " CTE: " << cte << " steering: " << steer_value <<  " speed: " << speed << " throttle: "<< throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h, &output_file](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    // close output file
    if (output_file.is_open()) {
      output_file.close();
    }

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
