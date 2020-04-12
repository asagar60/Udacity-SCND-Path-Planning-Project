## Path-Planning-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Project - 7: Path Planning Project.

This Project uses spline tool and frenet coordinates to predict other cars future distance and returns optimal lane

Overview
---
In this project we will implement path planning algorithm to safely navigate through the highway.

Here the traffic is driving at +-10 MPH of the 50 MPH speed limit. Car's localization ,sensor fusion data, and sparse map list of waypoints around the highway is also provided in real time.

Goal
---
The car will try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car will avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car will be able to make one complete loop around the 6946m highway.

The present algorithm considers the total acceleration and jerk , and tries to minimize and keep it below 10m/s^2 and 10m/s^3.

[//]: # (Image References)

[image1]: ./Results/7M.png "7M"
[image2]: ./Results/10M.png "10M"
[image3]: ./Results/15M.png "15M"

## Output

![Output_gif](./Results/Output_gif.gif)

As per Last Testing the car was able to navigate through the traffic without any incidents and crossed 15 Miles.

7 miles
![image1]

10 Miles
![image2]

15 miles
![image3]

### Video

[Video](./Results/Output.mkv)

### Implementation Strategy

Refer to [this](./Model-documentation/Model-Documentation-PATH-PLANNING-PROJECT.docx) document for detailed Implementation strategy.

## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) points
----

#### 1.1 Compilation  - The code compiles correctly.
The code compiles without any errors . The code has been refactored for better readability.

#### 2.1 Valid Trajectories - The car is able to drive at least 4.32 miles without incident.
The car is able to drive and navigate through the traffic . The car is able to drive for 4.32 miles at minimum and also crossed 15 miles mark (as per latest test results).

#### 2.2 Valid Trajectories - The car drives according to the speed limit.
The car stays within speed limit of 50MPH. for safety reasons , I chose 49.5 MPH.

#### 2.3 Valid Trajectories - Max Acceleration and Jerk are not Exceeded.
For the car to avoid Max Acceleration and Jerk, Incremental Approach is used. In this car speed will increase by a factor at each step.

#### 2.4 Valid Trajectories - Car does not have collisions.
Car avoids collisions by checking and verifying that the car which at front / left / right side will maintain a certain distance . On this basis the ego vehicle's speed is adjusted. Same goes for the Lane Selection.

#### 2.5 Valid Trajectories - The car stays in its lane, except for the time between changing lanes.
The Ego vehicle stays in the lane , unless it finds a better lane before switching the lanes

#### 2.6 Valid Trajectories - The car is able to change lanes
The Ego vehicle is able to smoothly change lanes when it makes sense to do so.


#### 3.1 Reflection



### Simulator Details

Download Simulator from [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Environment Details

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
