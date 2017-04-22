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
   * eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`

## Results
Based on the provided data set, my Unscented Kalman Filter will produce the below results. The x-position is shown as 'px', y-position as 'py', velocity in the x-direction is 'vx', while velocity in the y-direction is 'vy'. Residual error is calculated by mean squared error (MSE).

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.06908 |
|  py   | 0.07967 |
|  vx   | 0.16735 |
|  vy   | 0.20016 |

### Comparisons
Below I have also included the results of only running one sensor at a time (lidar or radar), as well as what the results of my previous [Extended Kalman Filter](https://github.com/mvirgo/Extended-Kalman-Filter) are for this dataset. As expected, the Unscented Kalman Filter that uses sensor fusion to combine lidar and radar measurements is the most accurate (lowest MSE) of the results.

| Input | UKF-Lidar | UKF-Radar | EKF-Fused | UKF-Fused |
| ----- | --------- | --------- | --------- | --------- |
|  px   |  0.09346  |  0.15123  |  0.13997  |  0.06908  |
|  py   |  0.09683  |  0.19708  |  0.66551  |  0.07967  |
|  vx   |  0.24238  |  0.20591  |  0.60388  |  0.16735  |
|  vy   |  0.31516  |  0.24436  |  1.62373  |  0.20016  |
