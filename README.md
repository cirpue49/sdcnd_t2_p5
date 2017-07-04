# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

[image1]: ./examples/kinematic_model.png
[image2]: ./examples/actuators.png
[image3]: ./examples/map.png
[image4]: ./examples/fitting.png
[image5]: ./examples/mpc.png
[image6]: ./examples/mpc2.png

### Motion Model: Assuming Kinematic Model

![alt text][image1]

### Actuators Constraint: Steering Angle and Throttle

Steering Angle [-30 ~ 30] (degree)

Throttle [-1 ~ 1]

![alt text][image2]


### Converting Coordinate from Global to Local

All information sent by simulator needs to be converted to local coordinate.

```
Eigen::VectorXd sub_x = e_ptsx - px * Eigen::VectorXd::Ones(6);
Eigen::VectorXd sub_y = e_ptsy - py * Eigen::VectorXd::Ones(6);
Eigen::VectorXd e_x = sub_x * cos(-psi) - sub_y * sin(-psi);
Eigen::VectorXd e_y = sub_x * sin(-psi) + sub_y * cos(-psi);
```

![alt text][image3]


### Fitting trajectory with 3rd polynomial

The reason why fitting with 3rd polynomial is that almost all roads or trajectories could fit with 3rd polynomial.

![alt text][image4]

### Applying MPC Alogorithm

Length(N) and Duration(dt) has to be chosen properly.

![alt text][image5]

MPC calculate the future states in a way that minimizing various kinds of errors.

![alt text][image6]


##### Errors 
Weighting Changing Steering Error to stabilize a car
```
// The part of the cost based on the reference state.
for (int i = 0; i < N; i++) {
  fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
}

// Minimize the use of actuators.
for (int i = 0; i < N - 1; i++) {
  fg[0] += CppAD::pow(vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i], 2);

}

// Minimize the value gap between sequential actuations.
for (int i = 0; i < N - 2; i++) {
  fg[0] += 500*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}


```

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

