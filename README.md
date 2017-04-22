# Unscented-Kalman-Filter
Udacity CarND Term 2, Project 2 - Unscented Kalman Filters

## Project Basics
In this project, I used C++ to write a program taking in radar and lidar data to track position using Unscented Kalman Filters, a more advanced and more accurate method than in my previous Extended Kalman Filter project.

The code will make a prediction based on the sensor measurement and then update the expected position. See files in the 'src' folder for the primary C++ files making up this project.

## Build instructions
Assuming you have 'cmake' and 'make' already:
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find some sample inputs in 'data/'.
   * eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Results
This project is still in-progress.

* Completed Steps:
   * Adding RMSE calculation to tools.cpp
   * Adding initialization code
   * Adding prediction code
* Upcoming Steps:
   * Adding update code for both radar and lidar
   * Tuning certain initialization paramaters to help reduce end RMSE and NIS
   * Adding visualizations (may be concurrent with above step)
