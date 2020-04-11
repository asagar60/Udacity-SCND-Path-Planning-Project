//
// Created by arun on 05/04/20.
//


#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "additional_helper_func.h"
using namespace std;

using std::vector;
using std::string;

int generate_trajectory(vector<Vehicle> active_predictions, int current_lane, double prev_size, double &ref_vel,
                    double car_d, double car_s, double end_path_s) {

    vector <double> lane_cost (3, 0.0);
    vector<string> new_states = successor_states(current_lane, lane_cost);
    int new_lane = -1;

    vector <double> lane_speed(3,ref_vel);

    if (active_predictions[0].position.compare("None")!=0) {

        if ((active_predictions[0].current_distance_from_ego > -10 &&
             active_predictions[0].current_distance_from_ego < 30) ||
                (active_predictions[0].future_distance_from_ego < 30)) {

            if (active_predictions[0].speed == MAX_SPEED) {
                lane_cost[current_lane] -= CAR_MAX_SPEED;
                lane_speed[current_lane] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

            } else {
               // cout<<"\n"<<"Car Speed :"<<active_predictions[0].speed;
                lane_cost[current_lane] += COLLISION;
                lane_speed[current_lane] = ref_vel > active_predictions[0].speed ? ref_vel - .224 : active_predictions[0].speed;

            }
        }else {
            lane_cost[current_lane] -= FREE_LANE;
            lane_speed[current_lane] =  ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

        }
    }else{


           lane_cost[current_lane] -= FREE_LANE;
           lane_speed[current_lane] =  ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

       }


    for (int i = 0; i < new_states.size(); ++i) {
        if (new_states[i].compare("LCL") == 0) {
            if (active_predictions[1].position.compare("None") != 0) {

                double check_car_s =
                        active_predictions[1].s + ((double) prev_size * 0.02 * active_predictions[1].speed);

                if ((active_predictions[1].current_distance_from_ego > -20 &&
                     active_predictions[1].current_distance_from_ego < 30) ||
                    (active_predictions[1].future_distance_from_ego < 30)) {

                    if (active_predictions[1].speed == MAX_SPEED) {
                        lane_cost[current_lane - 1] -= CAR_MAX_SPEED;
                        lane_speed[current_lane - 1] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;


                    } else {

                        lane_cost[current_lane - 1] += COLLISION;
                        lane_speed[current_lane - 1] = ref_vel > active_predictions[1].speed ? ref_vel - .224 : active_predictions[1].speed;

                    }
                }else {
                    lane_cost[current_lane - 1] -= FREE_LANE;
                    lane_speed[current_lane - 1] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

                }
            } else {
                lane_cost[current_lane - 1] -= FREE_LANE;
                lane_speed[current_lane - 1] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

            }
        } else if (new_states[i].compare("LCR") == 0) {
            if (active_predictions[2].position.compare("None") != 0) {

                double check_car_s =
                        active_predictions[2].s + ((double) prev_size * 0.02 * active_predictions[2].speed);

                if ((active_predictions[2].current_distance_from_ego > -20 &&
                     active_predictions[2].current_distance_from_ego < 30) ||
                    (active_predictions[2].future_distance_from_ego < 30)) {

                    if (active_predictions[2].speed == MAX_SPEED) {
                        lane_cost[current_lane + 1] -= CAR_MAX_SPEED;
                        lane_speed[current_lane + 1] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;


                    } else {

                        lane_cost[current_lane + 1] += COLLISION;
                        lane_speed[current_lane + 1] = ref_vel > active_predictions[2].speed ? ref_vel - .224 : active_predictions[2].speed;

                    }
                }
                else{
                    lane_cost[current_lane + 1] -= FREE_LANE;
                    lane_speed[current_lane + 1] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

                }
            }

                else {
                    lane_cost[current_lane + 1] -= FREE_LANE;
                    lane_speed[current_lane + 1] = ref_vel < MAX_SPEED  && ref_vel + .67 <= MAX_SPEED ? ref_vel + .67 : ref_vel;

                }
            }

        }


    vector<double>::iterator best_cost = std::min_element(lane_cost.begin(), lane_cost.end());
    int best_idx = std::distance(lane_cost.begin(), best_cost);


    if(lane_cost[best_idx] == lane_cost[current_lane]){
        new_lane = current_lane;
        ref_vel = lane_speed[new_lane];

        return new_lane;
    }
    else{

        new_lane = best_idx;
        ref_vel = lane_speed[new_lane];

        return new_lane;

    }

}


vector<string> successor_states(int current_lane, vector<double> &lane_cost) {
    vector<string> states;
    if (current_lane == 1){

        states.push_back("LCL");
        states.push_back("LCR");
    }
    else if (current_lane == 0){
        states.push_back("LCR");
        lane_cost[2] = INVALID_LANE;
    }

    else if(current_lane == 2) {
        states.push_back("LCL");
        lane_cost[0] = INVALID_LANE;
    }

    return states;
}


vector<Vehicle> predictions(vector<vector<double>> &sensor_fusion, int current_lane, double prev_size, double car_s, double end_path_s) {

    vector<Vehicle> active_predictions;
    active_predictions.push_back(Vehicle()); //front
    active_predictions.push_back(Vehicle());  // left
    active_predictions.push_back(Vehicle());  //right


    for (int i = 0; i < sensor_fusion.size(); i++) {

        float d = sensor_fusion[i][6];

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = sensor_fusion[i][5];

        double future_check_car_s = check_car_s+  ((double) prev_size * 0.02 * check_speed);
        for (int j = 0; j < 3; j++) {

            //check for vehicle ahead in current lane
            if ((j == current_lane) && ( d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2))) {

                    if (active_predictions[0].current_distance_from_ego > check_car_s - car_s  && future_check_car_s - end_path_s > 0){

                        active_predictions[0].current_distance_from_ego = check_car_s - car_s;
                        active_predictions[0].future_distance_from_ego = future_check_car_s - end_path_s ;
                        active_predictions[0].position = "front";
                        active_predictions[0].s = check_car_s;
                        active_predictions[0].speed = check_speed * 0.44;
                    }
            }
                //left car
            else if ((j == current_lane - 1 ) && (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2))) {

                    if (active_predictions[1].current_distance_from_ego > check_car_s - car_s && future_check_car_s - end_path_s > 0){

                        active_predictions[1].current_distance_from_ego = check_car_s - car_s;
                        active_predictions[0].future_distance_from_ego = future_check_car_s - end_path_s;
                        active_predictions[1].position = "left";
                        active_predictions[1].s = check_car_s;
                        active_predictions[1].speed = check_speed * 0.44;

                    }

            }
                //right car
            else if ((j == current_lane + 1) && (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) ){

                    if (active_predictions[2].current_distance_from_ego > check_car_s - car_s && future_check_car_s - end_path_s > 0){

                        active_predictions[2].current_distance_from_ego = check_car_s - car_s;
                        active_predictions[2].future_distance_from_ego = future_check_car_s - end_path_s;
                        active_predictions[2].position = "right";
                        active_predictions[2].s = check_car_s;
                        active_predictions[2].speed = check_speed * 0.44;

                    }
            }

        }
    }


    return active_predictions;
}