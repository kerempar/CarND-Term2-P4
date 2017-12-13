# PID Controller Project
Self-Driving Car Engineer Nanodegree Program


## Kerem Par
<kerempar@gmail.com>

---

## Introduction

In this project, a PID controller was implemented in C++ to maneuver the vehicle around the lake race track and the PID hyperparameters were tuned by applying the general processing flow as described in the previous lessons.

The simulator provides the cross track error (CTE) and the velocity (mph) in order the PID controller to compute the appropriate steering angle.

## Compilation

Code compiles without errors with cmake and make. 
The following change has been made to the original CMakeLists.txt. The line link_directories(/usr/local/Cellar/libuv/1.11.0/lib) was replaced by link_directories(/usr/local/Cellar/libuv/1.15.0/lib) because of the version of libuv installed.

## Implementation

The base algorithm follows PID procedure that was taught in the lessons.

###Hyperparameter tuning/optimization

`Twiddle` has been used for choosing the final hyperparameters (P, I, D coefficients). The twiddle implementation introduced in the lessons has been followed. It was started with target parameters set to 0 (`p={0, 0, 0}`), vector of potential changes to 1 (`dp={1,1,1}`) and the tolerance set to `0.001`. The twiddle has been run for a fixed throttle value set to `0.3`. The number of steps for a single run was chosen as `6,000` to cover at least one complete lap arounfd the track (depending on the speed of the car, it corresponds up to one and a half lap). The following table shows some significant combinations resulted from twiddle iterations. 


| Iteration |  K<sub>p</sub>  | K<sub>i</sub> | K<sub>d</sub> | 
| ----- | ------- | ------- | ------- |
|  7    |  0.109  | 0.0  | 5.13513  | 
|  26   |  0.154487  | 0.0  | 5.13513  | 
|  28   |  0.272453  | 0.0  | 4.97653  | 

Although twiddle produced slightly different values as number of iterations increased, while testing it was observed that the combination that was produced by iteration 7 to be more stable. That's why this combination (`0.109, 0.0, 5.13513`) has been used for further tests.

While performing twiddle, the `reset` command of the simulator has been used to restart the simulation from the beginning for each new run.    

To speedup the twiddle process, iterations are prematurely ended when the vehicle goes out of the track, it gets stuck on the barriers or hits some obstacle and stops. To detect those cases, certain conditions are checked for each step of the simulation (e.g. the cte is greater than `5.39`, speed is less than `0.5` and `10%` of the simulation steps of the current run have already been completed).

The twiddle mode can enabled by setting a global variable `twiddle_enabled` to true in the main program.
If `twiddle_enabled` is set to false, the program starts with `0.109, 0.0, 5.13513` values for the K<sub>p</sub>, K<sub>i</sub> and K<sub>d</sub> parameters, respectively.   


## Reflection

The folowing describes the effect each of the P, I, D components had in the implementation.

### P (Proportional)

P component provides the steering angle to be in proportion to the cross track error (cte). I paractice, it enables the car to turn enough towards the center line to reduce the cte.

K<sub>p</sub> paramater has been used as `0.109` (as twiddle suggested) in the final implementation.

The following video shows the when the PID controller running without P parameter. The steering angle does not change in proportion to cte and the vehicle eventually goes out of track. 

[![no P](./output_images/nop.png =550x350)](./video/PID_Control_Video_nop.mov)

### I (Integral)

I component provides the steering angle to be in proportion to the accumulated cross track error (cte) to overcome systematic bias in the system.

K<sub>i</sub> paramater has been used as `0` (as twiddle suggested) throughout the tests without any problems, It was concluded that there is no significant constant cross track error or a systematic bias exists in the system. On the other hand, whenever any nonzero values were tried for K<sub>i</sub>, the vehicle always went out of the track in a short time.

### D (Differential)

D component provides the steering angle to be in proportion to the rate of change in the cross track error (cte). In practice, it enables the car to approach to the center line more gracefully or gently to avoid the overshoot.
 
K<sub>d</sub> paramater has been used as `5.13513` (as twiddle suggested) in the final implementation.

The following video shows the when the PID controller running without D parameter. The steering angle continously changes back and forth of the center line (cannot stick to the center line) and the vehicle quickly goes out of track.

[![no D](./output_images/nod.png =550x350)](./video/PID_Control_Video_nod.mov)

## Simulation

After tuning the hyperparameters, I first tried the parameters with a constant throttle set to `0.3`. I verified that the vehicle can successfully complete a lap around the track safely with a maximum speed of `34 mph`. No tire left the drivable portion of the track surface. 

A complete lap of the vehicle can seen in the following video with this configuration.

[![fixed throttle 0.3](./output_images/35mph.png =550x350)](./video/PID_Control_Video_35mph.mov)

Then, I tried higher constant throttle values up to `0.7` to be able to see how fast the vehicle can still go safely with the same parameters. I observed that it goes slightly over curbs or lines for some curves with `0.7`. Finally I tried a variable throttle setting allowing the throttle between `0.3` and` 0.7`. I did not implement a separate pid controller for the throttle, however, I tried a simple setup which is inversely proportional to the steering angle, to slightly reduce the speed especially within the curves. I kept this final configuration and I observed a maximum speed of `71 mph`.    

A complete lap of the vehicle can seen in the following video with this configuration.

[![variable throttle 0.3-0.7](./output_images/70mph.png =550x350)](./video/PID_Control_Video_70mph.mov)


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


