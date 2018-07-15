# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# Discussion on work done
Rob Poyck

## The model
The MPC (Model Predictive Controller) consists of the following main steps:
 * Taking the vehicle state as starting variable
     * Position, x and y
     * Orientation, psi
     * Steering angle
     * Throttle
     * Speed (velocity)
 * Accepting a goal/ideal trajectory from the planner in the form of 2D coordinates of waypoints
     * ptsx and ptsy
 * Simulating actuator inputs
 * Predicting The resulting trajectory based on 
     * The motion model of the specific vehicle. The equations for the model:
         * x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
         * y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
         * psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
         * v_[t+1] = v[t] + a[t] * dt
         * cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
         * epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
     * The length of the trajectory that needs to be predicted (to be compliant to the road ahead and not only the current timestep)
     * The length of the timesteps between the predicted points on the trajectory (to define the 'resolution' of the trajectory)
 * Selecting the trajectory with the lowest cost based on:
     * Precision, i.e. deviation from the desired position and orientation
     * Constraints, such as:
         * The physical vehicle model
         * Maximum actuator values, e.g. maximum steering angle
     * Minimising actuator usage (to minimise constant correction/oscillation)
     * Minimising the value gap between sequential actuations (to smoothen the trajectory)
 * Send the first actuator values of this trajectory to the vehicle (throwing away the rest of the trajectory)
 * Next iteration starting from the top of this list
 


## Timestep Length and Elapsed Duration (N & dt)
The values were optimised to:  
 * N = 10 [-]
 * dt = 0.1 [s]
This means the planned trajectory / lookahead distance has a length of 10 sections 0.1 seconds, i.e. 1 [s].
Larger values of N lead to erratic behaviour of the path, where the difficulty in finding a singular trajectory can lead to large jumps and irregularities such as sudden loops in the trajectory because it wants to plan too far in the future. Smaller values of N will mean the lookahead distance is not large enough.  
Smaller values of dt (and consequently increasing N to maintain the same duration) lead to longer computational time without significant gain in precision.  

## Polynomial Fitting and MPC Preprocessing
The information coming from the simulator is pre-processed as follows:
 * The orientation and velocity after the duration of the latency are estimated using the vehicle motion model equations.
 * The waypoints coming from the simulator "planner" are processed as follows:
     * They arrive in the global CS and are converted to the vehicle CS for ease of computation (start position and orrientation are 0)
     * The x-coordinate is shifted to account for latency depending on vehicle velocity (ignoring the orientation change due to steering angle, which is why the y-shift is assumed to be 0 as seen from the vehicle)
 * After which a polynomial fit is made on these adjusted waypoints
 
## Model Predictive Control with Latency
As described in the previous section.  
Implementation of this can be found in:
 * main.cpp, lines:
     * 127-129
     * 145
 
 
---
# Original Udacity readme content

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
