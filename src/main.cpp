#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"


// for convenience
using json = nlohmann::json;

const double latency_ass = 0.100;		// Latency assumption //

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
	return "";
    } else if (b1 != string::npos && b2 != string::npos) {
	return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
	result += coeffs[i] * pow(x, i);
    }
    return result;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
	A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
	for (int i = 0; i < order; i++) {
	    A(j, i + 1) = A(j, i) * xvals(j);
	}
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;
    
    /*double latency_ass = 0.100;		// Latency assumption //*/	

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
    {
	// "42" at the start of the message means there's a websocket message event.
	// The 4 signifies a websocket message
	// The 2 signifies a websocket event
	string sdata = string(data).substr(0, length);
	cout << sdata << endl;
	if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
	    string s = hasData(sdata);
	    if (s != "") {
		auto j = json::parse(s);
		string event = j[0].get<string>();
		if (event == "telemetry") {
		    // j[1] is the data JSON object
		    vector<double> ptsx = j[1]["ptsx"];
		    vector<double> ptsy = j[1]["ptsy"];
		    double psi = j[1]["psi"];
// 		    double psi_unity = j[1]["psi_unity"];
		    double px = j[1]["x"]; // x-location of vehicle //
		    double py = j[1]["y"]; // y-location of vehicle //
		    double delta = j[1]["steering_angle"];
		    double a = j[1]["throttle"];
		    double v = j[1]["speed"];
		    
		    /*
		    * TODO: Calculate steering angle and throttle using MPC.
		    *
		    * Both are in between [-1, 1].
		    *
		    */
		    
		    // Adjust for latency //
		    psi += - v / Lf * delta * latency_ass;
		    v += a * latency_ass;
		    
		    // Transform the waypoints from map CS to vehicle CS // 
// 		    vector<double> ptsx_veh(ptsx.size());
// 		    vector<double> ptsy_veh(ptsx.size());
		    Eigen::VectorXd ptsx_veh(ptsx.size());
		    Eigen::VectorXd ptsy_veh(ptsx.size());
		    for (unsigned int i=0; i < ptsx.size(); i++)
		    {
			// Correct for translation of the origin of the CS// 
			double ptsx_trans = ptsx[i] - px;
			double ptsy_trans = ptsy[i] - py;
			// Correct for rotation around the z-axis (yaw) //
			ptsx_veh[i] = ptsx_trans * cos(-psi) - ptsy_trans * sin(-psi);
			ptsy_veh[i] = ptsx_trans * sin(-psi) + ptsy_trans * cos(-psi);
			// Adjust x position for latency (y value unchanged because steering angle is assumed to be small) //
			ptsx_veh[i] -= v * latency_ass;
		    }
		    
		    // Find the third degree polynomial best fit through the way-points //
		    Eigen::VectorXd coeffs = polyfit(ptsx_veh, ptsy_veh, 3);
		    
		    // Distance between the the track start and current location //
		    double cte = polyeval(coeffs, 0);
		    // Distance between the the track start angle and 0 //
		    double epsi = -atan(coeffs[1]);
		    
		    // State vector where x, y and psi are 0 because it was transformed to vehicle CS //
		    Eigen::VectorXd state_adjusted(6);
		    state_adjusted << 0.0, 0.0, 0.0, v, cte, epsi;
		    
		    // Solve using the MPC //
		    auto vars = mpc.Solve(state_adjusted, coeffs);
		    
		    double steer_value = vars[0];
		    // Multiply the steering value by -1.0 because of the awkward clockwise-positive definition of the simulator //
		    steer_value *= -1.0;
		    double throttle_value = vars[1];

		    json msgJson;
		    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
		    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
		    msgJson["steering_angle"] = steer_value;
		    msgJson["throttle"] = throttle_value;

		    //Display the MPC predicted trajectory 
		    vector<double> mpc_x_vals;
		    vector<double> mpc_y_vals;
		    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
		    // the points in the simulator are connected by a Green line
		    for (unsigned int i=2; i < vars.size(); i+=2) 
		    {
			mpc_x_vals.push_back(vars[i]);
			mpc_y_vals.push_back(vars[i+1]);
		    }
		    msgJson["mpc_x"] = mpc_x_vals;
		    msgJson["mpc_y"] = mpc_y_vals;

		    //Display the waypoints/reference line
		    vector<double> next_x_vals;
		    vector<double> next_y_vals;
		    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
		    // the points in the simulator are connected by a Yellow line
		    for (double i=0.0; i < 80.0; i += 5.0)
		    {
			next_x_vals.push_back(i);
			next_y_vals.push_back(polyeval(coeffs, i));
		    }
		    msgJson["next_x"] = next_x_vals;
		    msgJson["next_y"] = next_y_vals;


		    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		    std::cout << msg << std::endl;
		    // Latency
		    // The purpose is to mimic real driving conditions where
		    // the car does actuate the commands instantly.
		    //
		    // Feel free to play around with this value but should be to drive
		    // around the track with 100ms latency.
		    //
		    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
		    // SUBMITTING.
		    this_thread::sleep_for(chrono::milliseconds(100));
		    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}
	    } else {
		// Manual driving
		std::string msg = "42[\"manual\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	    }
	}
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
	const std::string s = "<h1>Hello world!</h1>";
	if (req.getUrl().valueLength == 1) {
	    res->end(s.data(), s.length());
	} else {
	    // i guess this should be done more gracefully?
	    res->end(nullptr, 0);
	}
    });

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
