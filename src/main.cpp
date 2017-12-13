#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits>

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

const double max_steer_angle = 1;
const double min_steer_angle = -1;

// twiddle related variables
std::vector<double> p =  {0, 0, 0};
std::vector<double> dp = {1, 1, 1};
const bool twiddle_enabled = false;
const int n = 6000;
const double tol = 0.001;
int it = 0;
double best_err = std::numeric_limits<double>::max();
int i = 0;
int twiddle_state = 0;
bool reset_flag = false;
double err = 0;
int num_runs = 0;
int run_steps = 0;
double best_Kp, best_Ki, best_Kd;


void twiddle(PID &pid, double &err)
{
  cout << "Twiddle..." << endl;
  
  double sum_dp = 0;
  
  cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
  cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
  
  for (int i=0; i < dp.size(); i++) {
    sum_dp += dp[i];
  }
  cout << "sum_dp: " << sum_dp << endl;
  
  while (sum_dp > tol) {
    
    cout << "iteration: " << it << " best error: " << best_err << endl;
    
    if (i >= p.size()) {it++; i=0;}
    cout << "i: " << i << endl;
    cout << "twiddle state: " << twiddle_state << endl;
    
    switch (twiddle_state)  {
      case 0:
        
        cout << "p[i] += dp[i]" << endl;
        p[i] += dp[i];
        cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
        cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
        // check next run
        pid.Init(p[0], p[1], p[2]);
        err = 0;
        run_steps = 0;
        // reset simulator
        reset_flag = true;
        twiddle_state = 1;
        return;
        
      case 1:
        
        if (err < best_err) {
          best_err = err;
          best_Kp = p[0]; best_Ki = p[1]; best_Kd= p[2];
          cout << "dp[i] *= 1.1" << endl;
          dp[i] *= 1.1;
          cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
          cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
          twiddle_state=0;
          i++;
          break;
        }
        else {
          cout << "p[i] -= 2 * dp[i]" << endl;
          p[i] -= 2 * dp[i];
          cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
          cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
          // check next run
          pid.Init(p[0], p[1], p[2]);
          err = 0;
          run_steps = 0;
          // reset simulator
          reset_flag = true;
          twiddle_state=2;
          return;
          break;
        }
        
      case 2:
        
        if (err < best_err) {
          best_err = err;
          best_Kp = p[0]; best_Ki = p[1]; best_Kd= p[2];
          cout << "dp[i] *= 1.1" << endl;
          dp[i] *= 1.1;
          cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
          cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
        }
        else {
          cout << "p[i] += dp[i]" << endl;
          p[i] += dp[i];
          cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
          cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
          
          cout << "dp[i] *= 0.9" << endl;
          dp[i] *= 0.9;
          cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2]: " << p[2] << endl;
          cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2]: " << dp[2] << endl;
        }
        twiddle_state=0;
        i++;  // next parameter
        break;
        
    } // switch
  } // while sum_dp > tol
} // twiddle


int main()
{
  uWS::Hub h;

  PID pid;
  
  // Initialize the pid variable.
  if (twiddle_enabled)
    pid.Init(0.0, 0.0, 0.0);
  else
    pid.Init(0.109, 0.0, 5.13513); // result of twiddle iteration 7 (throttle 0.3)
    //pid.Init(0.154487, 0.0, 5.13513); // result of twiddle iteration 26 (throttle 0.3)
    //pid.Init(0.27453, 0.0, 4.97653); // result of twiddle iteration 28 (throttle 0.3)
    
  run_steps = 0;
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle = 0.3;
          
          run_steps++;
          
          // calcuate steering value
          
          pid.UpdateError(cte);
          
          steer_value = pid.TotalError();
          
          // the steering value is [-1, 1]
          if (steer_value > max_steer_angle)
            steer_value = max_steer_angle;
          if (steer_value < min_steer_angle)
            steer_value = min_steer_angle;
          
          // set throttle between 0.3 and 0.8
          throttle = ((1 - fabs(steer_value)) * 0.5) + 0.3;
          
          // DEBUG
          std::cout << run_steps << " ******* CTE: " << cte << " Steering Value: " << steer_value << " Throttle: " << throttle << " *******" << std::endl;
          
          if (twiddle_enabled) {
            std::cout << "Runs: " << num_runs << " iteration: " << it << " best error: " << best_err << std::endl;
            std::cout << "best Kp: " << best_Kp << " best Ki: " << best_Ki << " best Kd: " << best_Kd << std::endl;
            std::cout << "Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;

            if ((fabs(cte) > 5.35 && run_steps>n/10) || (speed < 1.0 && run_steps>n/10)) {
              // if already gone out of road, no need to continue, finish the run prematurely to speedup twiddle
              for (; run_steps<n; run_steps++)
                err += pow(cte, 2);
            }
            else {
              err += pow(cte, 2);
              cout << "Error: " << err << endl;
            }
            if (run_steps >= n) {
              // calculate average error and call twiddle
              err = err / run_steps;
              twiddle(pid, err);
            }
          }
          
          if (reset_flag) {
            std::string msg = "42[\"reset\", {}]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            num_runs++;
            reset_flag = false;
          }
          else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
              
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
