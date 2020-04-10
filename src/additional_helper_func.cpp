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



int generate_trajectory(vector<Vehicle> active_predictions, string &state, int current_lane, double prev_size, double &ref_vel, double car_d, double car_s, double end_path_s){

    int new_lane;
    new_lane = lane_trajectory(active_predictions, current_lane, state , prev_size, ref_vel, car_d, car_s, end_path_s);

    return  new_lane;
}

int lane_trajectory(vector<Vehicle> active_predictions, int current_lane, string &state, int prev_size, double &ref_vel,
                    double car_d, double car_s, double end_path_s) {
    vector<string> new_states = successor_states(current_lane);
    int new_lane = -1;
    vector <double> lane_cost (3, 0.0);
    vector <double> lane_speed(3,0.0);

    if (active_predictions[0].position.compare("None")!=0) {

        cout << "\n" << "Inside prediction 0";
        //can use distance from ego
        // double check_car_s = active_predictions[0].s + ((double) prev_size * 0.02 * active_predictions[0].speed);

        if ((active_predictions[0].current_distance_from_ego > -10 &&
             active_predictions[0].current_distance_from_ego < 30) ||
            (active_predictions[0].future_distance_from_ego > -10 &&
             active_predictions[0].future_distance_from_ego < 30)) {

            cout << "\n" << "Inside if function of prediction 0";

            if (active_predictions[0].speed == MAX_SPEED) {
                lane_cost[current_lane] -= CAR_MAX_SPEED;
                lane_speed[current_lane] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                cout << "\n";
                cout << "MAX Speed Detected" << endl;
                cout << "For Lane :" << current_lane << " Updated Lane cost to :" << lane_cost[current_lane]
                     << "and lane speed to :" << lane_speed[current_lane] << endl;

            } else {
                lane_cost[current_lane] += COLLISION;
                lane_speed[current_lane] = ref_vel > 0 ? ref_vel - .224 : 0;
                cout << "\n";
                cout << "Collision Detected" << endl;
                cout << "For Lane :" << current_lane << " Updated Lane cost to :" << lane_cost[current_lane]
                     << "and lane speed to :" << lane_speed[current_lane] << endl;

            }
        }else{
            lane_cost[current_lane] -= FREE_LANE;
            lane_speed[current_lane] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
            cout<<"\n";
            cout<<"Free Lane Detected"<<endl;
            cout<<"For Lane :"<<current_lane<<" Updated Lane cost to :"<<lane_cost[current_lane]<< "and lane speed to :"<<lane_speed[current_lane]<<endl;
        }
    }else{


           lane_cost[current_lane] -= FREE_LANE;
           lane_speed[current_lane] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
           cout<<"\n";
           cout<<"Free Lane Detected"<<endl;
           cout<<"For Lane :"<<current_lane<<" Updated Lane cost to :"<<lane_cost[current_lane]<< "and lane speed to :"<<lane_speed[current_lane]<<endl;
       }

       /**
       else{



       }
        if ((check_car_s > car_s) && (check_car_s - car_s < 30)){

            cout<<"\n"<<"Inside if function of prediction 0";

            if(active_predictions[0].speed == MAX_SPEED){
                //lane_cost[current_lane] -= CAR_MAX_SPEED;
                lane_speed[current_lane] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                cout<<"\n";
                cout<<"MAX Speed Detected"<<endl;
                cout<<"For Lane :"<<current_lane<<" Updated Lane cost to :"<<lane_cost[current_lane]<< "and lane speed to :"<<lane_speed[current_lane]<<endl;

            }
            else {
                lane_cost[current_lane] += COLLISION;
                lane_speed[current_lane] = ref_vel > 0 ? ref_vel - .224 : 0;
                cout<<"\n";
                cout<<"Collision Detected"<<endl;
                cout<<"For Lane :"<<current_lane<<" Updated Lane cost to :"<<lane_cost[current_lane]<< "and lane speed to :"<<lane_speed[current_lane]<<endl;

            }
        }else{
            lane_cost[current_lane] -= FREE_LANE;
            lane_speed[current_lane] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
            cout<<"\n";
            cout<<"Free Lane Detected"<<endl;
            cout<<"For Lane :"<<current_lane<<" Updated Lane cost to :"<<lane_cost[current_lane]<< "and lane speed to :"<<lane_speed[current_lane]<<endl;
        }
    }
    else{
        lane_cost[current_lane] -= FREE_LANE;
        lane_speed[current_lane] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
        cout<<"\n";
        cout<<"Free Lane Detected"<<endl;
        cout<<"For Lane :"<<current_lane<<" Updated Lane cost to :"<<lane_cost[current_lane]<< "and lane speed to :"<<lane_speed[current_lane]<<endl;

    }
*/



    for (int i = 0; i < new_states.size(); ++i) {
        if (new_states[i].compare("LCL") == 0) {
            if (active_predictions[1].position.compare("None") != 0) {
                cout << "\n" << "Inside prediction 1";
                double check_car_s =
                        active_predictions[1].s + ((double) prev_size * 0.02 * active_predictions[1].speed);

                if ((active_predictions[1].current_distance_from_ego > -10 &&
                     active_predictions[1].current_distance_from_ego < 30) ||
                    (active_predictions[1].future_distance_from_ego > -10 &&
                     active_predictions[1].future_distance_from_ego < 30)) {


                    cout << "\n" << "Inside if function of prediction 1";
                    if (active_predictions[1].speed == MAX_SPEED) {
                        lane_cost[current_lane - 1] -= CAR_MAX_SPEED;
                        lane_speed[current_lane - 1] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;

                        cout << "\n";
                        cout << "MAX Speed Detected" << endl;
                        cout << "For Lane :" << current_lane - 1 << " Updated Lane cost to :"
                             << lane_cost[current_lane - 1] << "and lane speed to :" << lane_speed[current_lane - 1]
                             << endl;

                    } else {
                        lane_cost[current_lane - 1] += COLLISION;
                        lane_speed[current_lane - 1] = ref_vel > 0 ? ref_vel - .224 : 0;
                        cout << "\n";
                        cout << "Collision Detected" << endl;
                        cout << "For Lane :" << current_lane - 1 << " Updated Lane cost to :"
                             << lane_cost[current_lane - 1] << "and lane speed to :" << lane_speed[current_lane - 1]
                             << endl;
                    }
                }else{
                    lane_cost[current_lane - 1] -= FREE_LANE;
                    lane_speed[current_lane - 1] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                    cout << "\n";
                    cout << "Free Lane Detected" << endl;
                    cout << "For Lane :" << current_lane - 1 << " Updated Lane cost to :" << lane_cost[current_lane - 1]
                         << "and lane speed to :" << lane_speed[current_lane - 1] << endl;

                }
                /**
                else if ((check_car_s < car_s) && (car_s - check_car_s) < 30) {
                    lane_cost[current_lane - 1] += COLLISION;
                    lane_speed[current_lane - 1] = ref_vel > 0 ? ref_vel - .224 : 0;

                    cout<<"\n";
                    cout<<"Collision Detected"<<endl;
                    cout<<"For Lane :"<<current_lane-1<<" Updated Lane cost to :"<<lane_cost[current_lane-1]<< "and lane speed to :"<<lane_speed[current_lane-1]<<endl;

                    //ref_vel -= .224;
                }else{
                    lane_cost[current_lane-1] -= FREE_LANE;
                    lane_speed[current_lane-1] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                    cout<<"\n";
                    cout<<"Free Lane Detected"<<endl;
                    cout<<"For Lane :"<<current_lane-1<<" Updated Lane cost to :"<<lane_cost[current_lane-1]<< " and lane speed to :"<<lane_speed[current_lane-1]<<endl;
                }

                */
            } else {
                lane_cost[current_lane - 1] -= FREE_LANE;
                lane_speed[current_lane - 1] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                cout << "\n";
                cout << "Free Lane Detected" << endl;
                cout << "For Lane :" << current_lane - 1 << " Updated Lane cost to :" << lane_cost[current_lane - 1]
                     << "and lane speed to :" << lane_speed[current_lane - 1] << endl;


            }
        } else if (new_states[i].compare("LCR") == 0) {
            if (active_predictions[2].position.compare("None") != 0) {

                cout << "\n" << "Inside prediction 2";
                double check_car_s =
                        active_predictions[2].s + ((double) prev_size * 0.02 * active_predictions[2].speed);

                if ((active_predictions[2].current_distance_from_ego > -10 &&
                     active_predictions[2].current_distance_from_ego < 30) ||
                    (active_predictions[2].future_distance_from_ego > -10 &&
                     active_predictions[2].future_distance_from_ego < 30)) {

                    cout << "\n" << "Inside if function of prediction 2";
                    if (active_predictions[2].speed == MAX_SPEED) {
                        //lane_cost[current_lane + 1] -= CAR_MAX_SPEED;
                        lane_speed[current_lane + 1] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;


                        cout << "\n";
                        cout << "MAX Speed Detected" << endl;
                        cout << "For Lane :" << current_lane + 1 << " Updated Lane cost to :"
                             << lane_cost[current_lane + 1] << "and lane speed to :" << lane_speed[current_lane + 1]
                             << endl;


                    } else {
                        lane_cost[current_lane + 1] += COLLISION;
                        lane_speed[current_lane + 1] = ref_vel > 0 ? ref_vel - .224 : 0;
                        cout << "\n";
                        cout << "Collision Detected" << endl;
                        cout << "For Lane :" << current_lane + 1 << " Updated Lane cost to :"
                             << lane_cost[current_lane + 1] << "and lane speed to :" << lane_speed[current_lane + 1]
                             << endl;

                        //ref_vel -= .224;
                    }
                }
                else{
                    lane_cost[current_lane + 1] -= FREE_LANE;
                    lane_speed[current_lane + 1] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                    cout << "\n";
                    cout << "Free Lane Detected" << endl;
                    cout << "For Lane :" << current_lane + 1 << " Updated Lane cost to :" << lane_cost[current_lane + 1]
                         << "and lane speed to :" << lane_speed[current_lane + 1] << endl;
                }
            }
                    /**
                    else if ((check_car_s < car_s) && (car_s - check_car_s) < 30){
                        lane_cost[current_lane + 1] += COLLISION;
                        lane_speed[current_lane + 1] = ref_vel > 0 ? ref_vel - .224 : 0;
                        cout<<"\n";
                        cout<<"Collision Detected"<<endl;
                        cout<<"For Lane :"<<current_lane+1<<" Updated Lane cost to :"<<lane_cost[current_lane+1]<< "and lane speed to :"<<lane_speed[current_lane+1]<<endl;

                        //ref_vel -= .224;
                    }else{
                        lane_cost[current_lane+1] -= FREE_LANE;
                        lane_speed[current_lane+1] =  ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                        cout<<"\n";
                        cout<<"Free Lane Detected"<<endl;
                        cout<<"For Lane :"<<current_lane+1<<" Updated Lane cost to :"<<lane_cost[current_lane+1]<< "and lane speed to :"<<lane_speed[current_lane+1]<<endl;
                    }
                    */

                else {
                    lane_cost[current_lane + 1] -= FREE_LANE;
                    lane_speed[current_lane + 1] = ref_vel < MAX_SPEED ? ref_vel + .224 : ref_vel;
                    cout << "\n";
                    cout << "Free Lane Detected" << endl;
                    cout << "For Lane :" << current_lane + 1 << " Updated Lane cost to :" << lane_cost[current_lane + 1]
                         << "and lane speed to :" << lane_speed[current_lane + 1] << endl;

                }
            }

        }


    vector<double>::iterator best_cost = std::min_element(lane_cost.begin(), lane_cost.end());
    int best_idx = std::distance(lane_cost.begin(), best_cost);


    state = "KL";

    if(lane_cost[current_lane-1]==lane_cost[current_lane] || lane_cost[current_lane+1]==lane_cost[current_lane]){

        new_lane = current_lane;
        ref_vel = lane_speed[new_lane];
        cout<<"Current Lane :"<<current_lane<<" ";
        cout<<"Best Lane :"<<new_lane<<endl;
        cout<<"Lane cost :"<<lane_cost[new_lane]<<endl;
        cout<<"Lane Speed :"<<lane_speed[new_lane]<<endl;

        return new_lane;
    }

    if(fabs(current_lane - best_idx) == 1){

        new_lane = best_idx;
        ref_vel = lane_speed[new_lane];

        cout<<"\n";
        cout<<"Current Lane :"<<current_lane<<" ";
        cout<<"Best Lane :"<<best_idx<<endl;
        cout<<"Lane cost :"<<lane_cost[best_idx]<<endl;
        cout<<"Lane Speed :"<<lane_speed[best_idx]<<endl;


        return new_lane;
    }else{
        ref_vel = lane_speed[current_lane];
        cout<<"\n";
        cout<<"Lane cost :"<<lane_cost[current_lane]<<endl;
        cout<<"Lane Speed :"<<lane_speed[current_lane]<<endl;

        return current_lane;
    }
}


vector<string> successor_states(int current_lane) {
    vector<string> states;
    if (current_lane == 1){
        //states.push_back("PLCL");
        //states.push_back("PLCR");
        states.push_back("LCL");
        states.push_back("LCR");
    }
    else if (current_lane == 0){
        states.push_back("LCR");
    }

    else if(current_lane == 2) {
        states.push_back("LCL");
    }

    return states;
}


/**
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
                    if (active_predictions[0].distance_from_ego > check_car_s - car_s){
                        active_predictions[0].distance_from_ego = check_car_s - car_s;
                    }
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
                    if (active_predictions[1].distance_from_ego > check_car_s - car_s){
                        active_predictions[1].distance_from_ego = check_car_s - car_s;
                    }
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
                    if (active_predictions[2].distance_from_ego > check_car_s - car_s){
                        active_predictions[2].distance_from_ego = check_car_s - car_s;
                    }
                    active_predictions[2].too_close = true;
                }
                continue;
            }
            //else if (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
             //   if (((check_car_s > car_s) && (check_car_s - car_s) < 100) ||
             //       ((check_car_s < car_s) && (car_s - check_car_s) < 20)) {
              //      free_lane = j;
              //      lane_cost[free_lane] += EMPTY_LANE;
              //  }
           // }
        }
    }
    return active_predictions;
}

 */

vector<Vehicle> predictions(vector<vector<double>> &sensor_fusion, int current_lane, double prev_size, double car_s, double end_path_s) {

    //vector<int> active_predictions(4, 0); //front, left, right, free lane
    vector<Vehicle> active_predictions;
    active_predictions.push_back(Vehicle()); //front
    active_predictions.push_back(Vehicle());  // left
    active_predictions.push_back(Vehicle());  //right

    //int free_lane;

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
                        active_predictions[0].d = d;
                        active_predictions[0].speed = check_speed;
                        active_predictions[0].too_close = true;
                    }
                   // active_predictions[0].too_close = true;
            }
                //left car
            else if ((j == current_lane - 1 ) && (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2))) {

                    if (active_predictions[1].current_distance_from_ego > check_car_s - car_s && future_check_car_s - end_path_s > 0){

                        active_predictions[1].current_distance_from_ego = check_car_s - car_s;
                        active_predictions[0].future_distance_from_ego = future_check_car_s - end_path_s;
                        active_predictions[1].position = "left";
                        active_predictions[1].s = check_car_s;
                        active_predictions[1].d = d;
                        active_predictions[1].speed = check_speed;
                        active_predictions[1].too_close = true;
                    }
                    //active_predictions[1].too_close = true;
            }
                //right car
            else if ((j == current_lane + 1) && (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) ){

                    if (active_predictions[2].current_distance_from_ego > check_car_s - car_s && future_check_car_s - end_path_s > 0){

                        active_predictions[2].current_distance_from_ego = check_car_s - car_s;
                        active_predictions[2].future_distance_from_ego = future_check_car_s - end_path_s;
                        active_predictions[2].position = "right";
                        active_predictions[2].s = check_car_s;
                        active_predictions[2].d = d;
                        active_predictions[2].speed = check_speed;
                        active_predictions[2].too_close = true;

                    }
                    //active_predictions[2].too_close = true;
            }

            //else if (d < (2 + 4 * j + 2) && d > (2 + 4 * j - 2)) {
            //   if (((check_car_s > car_s) && (check_car_s - car_s) < 100) ||
            //       ((check_car_s < car_s) && (car_s - check_car_s) < 20)) {
            //      free_lane = j;
            //      lane_cost[free_lane] += EMPTY_LANE;
            //  }
            // }
        }
    }

    //selecting best lane with possibility of less traffic
/**
    int best_idx = -1;
    double max_value = std::numeric_limits<double>::min();
    for (int i =0; i < active_predictions.size(); ++i){
         if (max_value < active_predictions[i].distance_from_ego){
                max_value = active_predictions[i].distance_from_ego;
                best_idx = i;
            }
    }

*/
    /**

    //Add free lane cost for most traffic free lane

    if ( best_idx == 0){
        lane_cost[current_lane] = lane_cost[current_lane] - FREE_LANE;
    } else if(best_idx == 1){
        lane_cost[current_lane - 1] = lane_cost[current_lane -1] - FREE_LANE;
    }else if (best_idx == 2){
        lane_cost[current_lane + 1] = lane_cost[current_lane + 1] - FREE_LANE;
    }

     */

    return active_predictions;
}

/**
int keep_lane_trajectory(vector<Vehicle> active_predictions, int current_lane, string &state , int prev_size, double &ref_vel, double car_d, double car_s) {

    vector<string> new_states = successor_states(state, current_lane);
    int new_lane = -1;

    double check_car_s = active_predictions[0].s + ((double) prev_size * 0.02 * active_predictions[0].speed);

    if ((check_car_s > car_s) && (check_car_s - car_s) < 30){
        lane_cost[current_lane] += COLLISION;
        ref_vel -= .224;
    }

    for (int i = 0; i < new_states.size(); ++i) {
        if (new_states[i].compare("LCL") == 0) {

        } else if (new_states[i].compare("LCR") == 0) {

        }

    }

    vector<double>::iterator best_cost = std::min_element(lane_cost.begin(), lane_cost.end());
    int best_idx = std::distance(lane_cost.begin(), best_cost);

    if (best_idx == 0) {
        state = "KL";
        new_lane = current_lane;
    }else if(best_idx == 1){
        state = "LCL";
        new_lane = current_lane -1;
    }
    else if(best_idx == 2) {
        state = "LCR";
        new_lane = current_lane + 1;
    }

    return new_lane;
}


 */

/**
vector<Vehicle> lane_change_trajectory(){

}

 */



/**
int Lane_change(vector<Vehicle> active_predictions, string &state, int current_lane, int prev_size, double ref_vel, double car_d, double car_s) {

        int best_lane = -1;
        double car_dist = 25;
        int trajectory = generate_trajectory(active_predictions, state, current_lane, prev_size, ref_vel,  car_d,  car_s);
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
        return trajectory;
}
*/

/**
int lane_trajectory(vector<Vehicle> active_predictions, int current_lane, string &state, int prev_size, double &ref_vel,
                     double car_d, double car_s) {
    vector<string> new_states = successor_states(state, current_lane);
    int new_lane = -1;

    if (active_predictions[0].position.compare("None")!=0){

        double check_car_s = active_predictions[0].s + ((double) prev_size * 0.02 * active_predictions[0].speed);

        if ((check_car_s > car_s) && (check_car_s - car_s) < 30){

            if(active_predictions[0].speed == MAX_SPEED){
                lane_cost[current_lane] -= CAR_MAX_SPEED;
                if(ref_vel < MAX_SPEED){
                    ref_vel += .224;
                }
            }
            else {
                lane_cost[current_lane] += COLLISION;
                ref_vel -= .224;
            }
        }

        else{
            if(ref_vel < MAX_SPEED){
                ref_vel += .224;
            }
        }
    }
    else{
        lane_cost[current_lane] -= FREE_LANE;
    }


    for (int i = 0; i < new_states.size(); ++i) {
        if (new_states[i].compare("LCL") == 0 ){
            if(active_predictions[1].position.compare("None")!=0 ) {
                double check_car_s =
                        active_predictions[1].s + ((double) prev_size * 0.02 * active_predictions[1].speed);

                if ((check_car_s > car_s) && (check_car_s - car_s) < 30) {
                    if (active_predictions[1].speed == MAX_SPEED) {
                        lane_cost[current_lane - 1] -= CAR_MAX_SPEED;
                    } else {
                        lane_cost[current_lane - 1] += COLLISION;
                        //ref_vel -= .224;
                    }
                } else if ((check_car_s < car_s) && (car_s - check_car_s) < 30) {
                    lane_cost[current_lane - 1] += COLLISION;
                    //ref_vel -= .224;
                }
            }else {
                lane_cost[current_lane -1 ] -= FREE_LANE;
            }
        }else if (new_states[i].compare("LCR") == 0) {
            if (active_predictions[2].position.compare("None")!=0 ){
                double check_car_s = active_predictions[2].s + ((double) prev_size * 0.02 * active_predictions[2].speed);

                if ((check_car_s > car_s) && (check_car_s - car_s) < 30){
                    if(active_predictions[2].speed == MAX_SPEED){
                        lane_cost[current_lane + 1] -= CAR_MAX_SPEED;
                    }
                    else {
                        lane_cost[current_lane + 1] += COLLISION;
                        //ref_vel -= .224;
                    }
                }else if ((check_car_s < car_s) && (car_s - check_car_s) < 30){
                    lane_cost[current_lane + 1] += COLLISION;
                    //ref_vel -= .224;
                }
            }else{
                lane_cost[current_lane +1 ] -= FREE_LANE;
            }
        }

    }

    vector<double>::iterator best_cost = std::min_element(lane_cost.begin(), lane_cost.end());
    int best_idx = std::distance(lane_cost.begin(), best_cost);

    if (best_idx == 0) {
        new_lane = current_lane;
    }else if(best_idx == 1){
        new_lane = current_lane -1;
    }
    else if(best_idx == 2) {
        new_lane = current_lane + 1;
    }

    state = "KL";

    if(fabs(current_lane - new_lane) == 1){
        return new_lane;
    }else{
        return current_lane;
    }
}

 */
