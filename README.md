# Project 9: CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)
[image_pid_speed_control]: ./images/pid_speed_control_start.png
[image_pid_steering_control]: ./images/pid_steering_control_start.png
[image_pid_steering_control_full]: ./images/pid_steering_control_round.png

## Introduction
The purpose of this project was to implement a PID controller in C++ to maneuver the vehicle around a simulated race track. The Udacity simulator provides the actual velocity, the actual steering angle and the cross track error (CTE). To control the vehicles position on the track and the vehicle's speed I implemented two PID controllers. The first one controls the vehicle's steering angle and the second the vehicle's throttle position.

## PID Controller
#### P - Proportional
The P (proportional) component `-Kp * CTE` of the PID controller reacts directly to the cross track error (CTE). A huge CTE leads to strong steering intervention. If the Kp value is too large the controller overshoots the center line and starts to oscillate. If Kp is too low the controller reacts slowly on cross track errors. The vehicle tends to leave the track especially in sharp curves.

#### I - Integral
The I (integral) component `Ki * sum(cte) * dt` of the PID controller reduces constant bias and drifts from the center line. Large Ki values leads to an oscillating controller.

#### D - Differential
The D (differential) component `Kd * (CTE - prev_CTE) / dt` of the PID controller reduces the overshoots caused by the P component. It acts as a kind of damping mechanism. Small Kd values do not damp the system thus the controller tends to overshoot and oscillation. Too large Kd values leads to a slow reduction of the cross track error.

## PID Tuning Process
To tune the steering angle PID controller manually I applied the following steps.

1. Set all values (Kp, Ki, Kd) to zero.
1. Increase the Kp value until the controller starts to oscillate.
1. Increase the Kd value to damp the oscillation until the vehicle is able to drive one full round on the race track. In case the oscillation is still to high, slightly reduce the Kp value.
1. Finally increase the Ki value in small steps to reduce the bias and drifts especially incurves.

**Final PID Steering Controller**

`Kp=0.1 Ki=0.00025 Kd=3.5`

![PID steering controller][image_pid_steering_control]

Afterwards I tuned the throttle PID-Controller with the same approach as described above. Several parameter combinations showed that the best results will be achieved with an PI-Controller. Therefore, the Kd value has been set to zero.

**Final PID Speed Controller**

`Kp=0.5 Ki=0.000051 Kd=0.0 set_speed=30 mph`

![PID speed controller][image_pid_speed_control]

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
1. Make a build directory: `mkdir build && cd build`
1. Compile: `cmake .. && make`
1. Run it: `./pid`.

## Setup Xcode Development Environment

I developed the code by using the Xcode development environment. To generate the required project files the following steps are required.

1. Make a build directory: `mkdir build && cd build`
1. Generate the Xcode project files with `cmake -G Xcode ..`
1. Open the `PID.xcodeproj` file with Xcode and change the active scheme to `pid --> My Mac`
1. Done

## Code Style

The code has been written based on [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
