//
// Created by arun on 05/04/20.
//

#ifndef PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H
#define PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::vector;
using std::string ;

struct Vehicle{
    int vehicle_id = -1;
    double s;
    double d;
    double speed;
    bool too_close = false;
};

int Lane_change(vector <vector <double>> &, int ,bool &, bool &, bool &, bool &, bool &, double, double );
void check_too_close(vector<vector<double>> &sensor_fusion, int current_lane, bool &too_close,bool &car_ahead, bool &car_behind, bool &car_on_left, bool &car_on_right, double car_s, double prev_size);
int emptyLane(vector<vector<double>> &sensor_fusion, int current_lane);
vector<string> successor_states(string state, int current_lane);
vector<Vehicle> predictions(vector<vector<double>> &sensor_fusion, int current_lane, double prev_size, double car_s);
int Lane_change(vector<Vehicle> active_predictions, int current_lane, double car_s, double prev_size);


#endif //PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H
