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

int Lane_change(vector <vector <double>> &, int ,bool &, bool &, bool &, bool &, bool &, double, double );
void check_too_close(vector<vector<double>> &sensor_fusion, int current_lane, bool &too_close,bool &car_ahead, bool &car_behind, bool &car_on_left, bool &car_on_right, double car_s, double prev_size);
int emptyLane(vector<vector<double>> &sensor_fusion, int current_lane);


#endif //PATH_PLANNING_ADDITIONAL_HELPER_FUNC_H
