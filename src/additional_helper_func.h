//
// Created by arun on 05/04/20.
//

#ifndef PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H
#define PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H
//#include "helpers.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::vector;
using std::string ;

struct Vehicle{
    string position = "None";
    double s = -1;
    double d;
    double speed;
    double current_distance_from_ego = 500.0;
    double future_distance_from_ego = 500.0;
    bool too_close = false;
};



const double COLLISION = 600.0;
const double INVALID_LANE = 10000.0;
const double REDUCE_SPEED = 200.0;
const double FREE_LANE = 500.0;
const double CAR_MAX_SPEED = 300.0;
const double MAX_SPEED = 49.5;


//int Lane_change(vector <vector <double>> &, int ,bool &, bool &, bool &, bool &, bool &, double, double );
//void check_too_close(vector<vector<double>> &sensor_fusion, int current_lane, bool &too_close,bool &car_ahead, bool &car_behind, bool &car_on_left, bool &car_on_right, double car_s, double prev_size);
//int emptyLane(vector<vector<double>> &sensor_fusion, int current_lane);
vector<string> successor_states(int current_lane, vector <double> &lane_cost);
vector<Vehicle> predictions(vector<vector<double>> &sensor_fusion, int current_lane, double prev_size, double car_s,double end_path_s);
int Lane_change(vector<Vehicle> active_predictions, string &state, int current_lane, int prev_size, double ref_vel, double car_d, double car_s);
int generate_trajectory(vector<Vehicle> active_predictions, string &state, int current_lane, double prev_size, double &ref_vel, double car_d, double car_s, double end_path_s);
int lane_trajectory(vector<Vehicle> active_predictions, int current_lane, string &state , int prev_size, double &ref_vel, double car_d, double car_s, double end_path_s) ;
//int lane_change_left_trajectory(vector<Vehicle> active_predictions, int current_lane, string &state , int prev_size, double ref_vel, double car_d, double car_s);
//int lane_change_right_trajectory(vector<Vehicle> active_predictions, int current_lane, string &state , int prev_size, double ref_vel, double car_d, double car_s);

#endif //PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H
