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
   
## Initialization paramaters
This project included the option to tune various initialization parameters to improve the final calculated RMSE (root mean squared error). In the `ukf.cpp` file, I noted which portions I tuned, which included `std_a`, `std_yawd`, the initialized `x_` (separately for Radar and Lidar) and `P_` (also separate for each sensor type).

* `std_a_`: process noise standard deviation longitudinal acceleration in m/s^2. I settled on a value of 1 here, as for a bike (which is what we are detecting in this project) I would not expect very high acceleration.
* `std_yawd_`: process noise standard deviation yaw acceleration in rad/s^2. A big would have to swerve pretty heavily for a big acceleration in yaw, so I settled for .5 radians here.
* Radar
  * `x_`: Most of this is just an equation for the state space based off the first radar measurement. The middle value, for 'v', can be tuned. I set this to a value of 4 m/s as a quick Google search led me to an average bike speed of 15.5 km/h, which is just over 4 m/s
  * `P_`: The diagonal of the matrix are the variances for each value within the `x_` state space (px, py, v, yaw, yawd). Radar feeds in rho, phi and rhodot. We are given the standard deviation of each of these, and the square of the standard deviation is the variance. Although this is not the exact right value (given its in a different state space), I used the square of the given standard deviations to calculate reasonable values for `P_` (other than for the middle value of 'v' which I just set to 1.
* Lidar
  * `x_`: The first two values are filled by the 'px' and 'py' lidar measurements. I chose 4 m/s for 'v' in the same way as for radar. I chose yaw of .5 as a reasonable estimate of a beginning turn, with yawd as 0 given no expected big swerves at the start.
  * `P_`: We are given the standard deviations for px and py from the lidar measurements, so I squared these to feed in the respective variances to the matrix. I just used 1 for the other variances along the diagonal as a reasonable beginning value.

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
