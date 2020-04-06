//
// Created by arun on 05/04/20.
//

#include "additional_helper_func.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::vector;
using std::string;

/**
int Lane_change(vector<vector<double>> &sensor_fusion, int current_lane, bool &too_close, bool &car_ahead, bool &car_behind, bool &car_on_left, bool &car_on_right, double car_s, double prev_size) {

    int best_lane = -1;

    // check if there is an empty lane

    for (int i = 0; i < sensor_fusion.size(); i++) {

        float d = sensor_fusion[i][6];
        for (int j = 0; j < 3; ++j) {

            if (j== current_lane){ continue;}

            if (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                double car_dist = 50;

                check_car_s += ((double) prev_size * 0.02 * check_speed);
                if ((check_car_s > car_s) && (check_car_s - car_s) > car_dist){
                        car_dist = check_car_s - car_s;
                            best_lane = j;

                        }
                }
            }
    }

    return best_lane;
}

*/


/**
int Lane_change(vector<vector<double>> &sensor_fusion, int current_lane, bool &too_close, bool &car_ahead, bool &car_behind, bool &car_on_left, bool &car_on_right, double car_s, double prev_size) {

    int best_lane = -1;

    if (car_ahead == true){
        //check for left and right side

        if (car_on_left != true && current_lane > 0){
            best_lane = current_lane - 1;
        }
        else if(car_on_right != true && current_lane == 1){
            best_lane = current_lane + 1;
        }
        else {
            best_lane = current_lane;
        }
    } else{

        if ( current_lane != 1 ) { // if we are not on the center lane.
            if ( ( current_lane == 0 && !car_on_right ) || ( current_lane == 2 && !car_on_left ) ) {
                best_lane = 1; // Back to center.
            }
        }

    }
    return best_lane;
}


*/


void check_too_close(vector<vector<double>> &sensor_fusion, int current_lane, bool &too_close, bool &car_ahead, bool &car_behind, bool &car_on_left, bool &car_on_right, double car_s, double prev_size) {

    for (int i = 0; i < sensor_fusion.size(); i++) {

        float d = sensor_fusion[i][6];

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += ((double) prev_size * 0.02 * check_speed);

        for (int j = 0; j < 3; j++) {

            if (j == current_lane && d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {

                if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {

                    too_close = true;
                    car_ahead = true;
                }
                //else if ((check_car_s < car_s) && fabs(check_car_s - car_s) < 10) {
                    //too_close = true;
                 //   car_behind = true;
                //}
            } else if ((current_lane - j) ==-1 && d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)){

                if (((check_car_s > car_s) && (check_car_s - car_s) < 30) || ((check_car_s < car_s) && (car_s - check_car_s) < 30)){
                    car_on_left = true;
                }

            } else if((current_lane - j) == 1 && d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)){

                if (((check_car_s > car_s) && (check_car_s - car_s) < 30) || ((check_car_s < car_s) && (car_s - check_car_s) < 30)){
                    car_on_right = true;
                }

            }
        }
    }
}

int emptyLane(vector<vector<double>> &sensor_fusion, int current_lane){

    int cars_in_lane0 = 0;
    int cars_in_lane1 = 0;
    int cars_in_lane2 = 0;

    for(int i=0; i < sensor_fusion.size(); ++i){
        float d = sensor_fusion[i][6];

        if (d < (4) && d > (0)){
            ++cars_in_lane0;
            }
        else if(d < (8) && d > (4)){
            ++cars_in_lane1;
        }
        else if(d < (12) && d > (8)) {
            ++cars_in_lane2;
        }
    }

    if ((cars_in_lane0 == 0) && (current_lane == 1)){
        return 0;
    }
    else if ((cars_in_lane1 == 0) && (fabs(current_lane - 1)==1)){
        return 1;
    }
    else if ((cars_in_lane2 == 0) && (current_lane == 1)){
        return 2;
    }

    else{
        return -1;
    }
}

vector<string> successor_states(string state, int current_lane){
    vector<string> states;
    if (state.compare("KL")==0){
        states.push_back("PLCL");
        states.push_back("PLCR");
        //states.push_back("LCL");
        //states.push_back("LCR");
    }
    else if (state.compare("PLCL")==0){
        if (current_lane > 0){
            //check lane availability
            states.push_back("PLCL");
            states.push_back("LCL");
        }

    }
    else if(state.compare("PLCR")==0){
        if(current_lane < 2){
            //check lane availability
            states.push_back("PLCR");
            states.push_back("LCL");
        }
    }

    return states;
}



vector<double> lane_cost (3,100);
double SPEED_BELOW_MAX = 250;
double COLLISION = 1000;
double EMPTY_LANE = -500;
double SPEED_MAX = -250;
double RESET_LANE_COST = 100;
double MAX_SPEED = 49.5;

vector<Vehicle> predictions(vector<vector<double>> &sensor_fusion, int current_lane, double prev_size, double car_s) {

    //vector<int> active_predictions(4, 0); //front, left, right, free lane
    vector<Vehicle> active_predictions;
    active_predictions.push_back(Vehicle()); //front
    active_predictions.push_back(Vehicle());  // left
    active_predictions.push_back(Vehicle());  //right

    int free_lane;

    for (int i = 0; i < sensor_fusion.size(); i++) {

        float d = sensor_fusion[i][6];

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += ((double) prev_size * 0.02 * check_speed);
        for (int j = 0; j < 3; j++) {

            //check for vehicle ahead in current lane
            if (j == current_lane && d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
                if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {
                    active_predictions[0].vehicle_id = i;
                    active_predictions[0].s = check_car_s;
                    active_predictions[0].d = d;
                    active_predictions[0].speed = check_speed;
                    active_predictions[0].too_close = true;
                }
                continue;
            }
            //left car
            else if (j == current_lane - 1 && d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
                if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                    ((check_car_s < car_s) && (car_s - check_car_s) < 15)) {
                    active_predictions[1].vehicle_id = i;
                    active_predictions[1].s = check_car_s;
                    active_predictions[1].d = d;
                    active_predictions[1].speed = check_speed;
                    active_predictions[1].too_close = true;
                }
                continue;
            }
            //right car
            else if (j == current_lane + 1 && d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
                if (((check_car_s > car_s) && (check_car_s - car_s) < 35) ||
                    ((check_car_s < car_s) && (car_s - check_car_s) < 15)) {
                    active_predictions[2].vehicle_id = i;
                    active_predictions[2].s = check_car_s;
                    active_predictions[2].d = d;
                    active_predictions[2].speed = check_speed;
                    active_predictions[2].too_close = true;
                }
                continue;
            } else if (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
                if (((check_car_s > car_s) && (check_car_s - car_s) < 100) ||
                    ((check_car_s < car_s) && (car_s - check_car_s) < 20)) {
                    free_lane = j;
                    lane_cost[free_lane] += EMPTY_LANE;
                }
            }
        }
    }
    return active_predictions;
}


int Lane_change(vector<Vehicle> active_predictions, int current_lane, double car_s, double prev_size) {

    int best_lane = -1;
    double car_dist = 25;
    if (active_predictions[0].too_close == true){
        //check for left and right side

        if (active_predictions[1].too_close != true && current_lane > 0){
            best_lane = current_lane - 1;
        }
        else if(active_predictions[2].too_close != true && current_lane == 1){
            best_lane = current_lane + 1;
        }
        else {
            best_lane = current_lane;
        }
    } else{

        if ( current_lane != 1 ) { // if we are not on the center lane.
            if ( ( current_lane == 0 && !active_predictions[2].too_close ) || ( current_lane == 2 && !active_predictions[1].too_close ) ) {
                best_lane = 1; // Back to center.
            }
        }

    }
    return best_lane;
}
/**

vector<Vehicle> keep_lane_trajectory(vector<Vehicle> active_predictions, int current_lane, string state = "KL"){

    vector<string> new_states = successor_states(state,current_lane);

    if(active_predictions[0].speed < MAX_SPEED){
        lane_cost[current_lane] += SPEED_BELOW_MAX;
    }
    if (active_predictions[0].speed == MAX_SPEED){
        lane_cost[current_lane] += SPEED_MAX;
    }

    for (int i=0; i < new_states.size(); ++i){
        if (new_states[i].compare("PLCL")==0){

        }
    }



}

vector<Vehicle> prepare_lane_change_trajectory(){


}

vector<Vehicle> lane_change_trajectory(){

}


vector<Vehicle> generate_trajectory(vector<vector<double>> &sensor_fusion, vector<Vehicle> active_predictions, vector<double> lane_cost, string state, int current_lane, double prev_size, double car_s){

    int new_lane= -1;
    vector<Vehicle> trajectory;

    //update lane cost if its free_lane;
    if (state.compare("KL")==0){
        trajectory = keep_lane_trajectory();
    }
    else if(state.compare("PLCL")==0  || state.compare("PLCR")==0){
        trajectory = prepare_lane_change_trajectory();
    }
    else if(state.compare("LCL")==0 || state.compare("LCR")==0){
        trajectory = lane_change_trajectory();
    }

    return  trajectory;
}

*/


