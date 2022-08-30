#include <iostream>
#include <uWS/uWS.h>
#include <thread>

#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

using nlohmann::json;
using std::string;
using std::vector;

int main() {
    uWS::Hub h;

    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        std::cout << sdata << std::endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (!s.empty()) {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    int waypnt_num = ptsx.size();
                    Eigen::VectorXd waypnt_x(waypnt_num);
                    Eigen::VectorXd waypnt_y(waypnt_num);

                    for (int i = 0; i < waypnt_num; ++i) {
                        double diff_x = ptsx[i] - px;
                        double diff_y = ptsy[i] - py;
                        waypnt_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
                        waypnt_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
                    }

                    auto coeffs = polyfit(waypnt_x, waypnt_y, 3);
                    double cte = polyeval(coeffs, 0);
                    double epsi = -atan(coeffs[1]);

                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, cte, epsi;

                    auto vars = mpc.Solve(state, coeffs);

                    double steer_value = vars[0];
                    double throttle_value = vars[1];

                    json msgJson;

                    msgJson["steering_angle"] = steer_value / deg2rad(25);
                    msgJson["throttle"] = throttle_value;

                    // Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    for (int i = 2; i < vars.size(); ++i) {
                        if (i % 2 == 0) {
                            mpc_x_vals.push_back(vars[i]);
                        } else {
                            mpc_y_vals.push_back(vars[i]);
                        }
                    }
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    // Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < 100; ++i) {
                        next_x_vals.push_back((double)i);
                        next_y_vals.push_back(polyeval(coeffs, (double)i));
                    }
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;

                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
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
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}