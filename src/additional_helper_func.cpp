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
    }
    else if (state.compare("PLCL")==0){
        if (current_lane > 0 ){
            //check lane availability
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if(state.compare("PLCR")==0){
        if (current_lane < 2){
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }

    return states;
}


