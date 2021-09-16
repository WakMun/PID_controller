# Controlling the Steering Angle

# Overview
In this project of Self-Driving Car Engineer Nanodegree Program from Udacity the main goal is to implement a 
simple [PID controller](https://en.wikipedia.org/wiki/PID_controller) to control a car driving autonomously around a race track in Udacity's simulator.

The core idea here is that the path has been already been planned for an autonomous car, a controller takes this path as
an input and decides upon the angle at which the car is to be steered so as to minimize the error from the decided path.
Due to inertia, imprecise motion the car and noisy measurements it is not possible to predict and control the position of the car with 
certainity. Hence there is always a deviation from the decided path. The job of the controller is to minimize this deviation without scarificing 
the speed. 

In this project, the [udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases) outputs the cross-track error that 
is fed to the PID control through a [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation. In response, the pid control decides the 
steering angle in the range [1,-1]. This is in physical terms corresponds to 25 degrees towards left or right. This command is fed to the simulator to steer the 
car in desired direction. This is done in a continuous loop. The second control parameter, the throttle, which determines the speed of the car, is fixed at 0.3.
Original repo from Udacity can be found [here](https://github.com/udacity/CarND-PID-Control-Project).


---

# Dependencies

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


# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

---

# Results


## Video Output
The final video output where the controller controls the car along the track can be seen [here](./videos/Tuned.mp4).

## Effect of each of the P, I, D components 

- The **proportional** component of the controller tries to steer the car toward the center line (against the cross-track error). This is the main component that tries to reduce the error while other components provide the damping. When propotional component is present alone, the car overshoots the desired trajectory and never settles as can be seen in the video: [./videos/Only_P.mp4](./videos/Only_P.mp4). In this case car keeps osciallating to and fro on the desired path. Specially on the curves the deviation becomes so big that car steers almost perpendicular to the road and overshoots to go out of the road on the opposite side.

- The **differential** component helps to dampen out the propotional component by reducing its response if the error is decreasing. On the other hand, if the error is increasing it enhances the response of propotional component. This has an effect of trend to overshoot the center line by smoothing the approach to it. An example video where this component is used alone is [./videos/Only_D.mp4](./videos/Only_D.mp4).

- The **integral** component is designed to remove the systematic bias that can can prevent the error to be completely eliminated. If used alone, it makes the car to 
go in circles. In the case of the simulator, no bias is present. An example video where this component is used alone is [./videos/Only_I.mp4](./videos/Only_I.mp4).


## Tuning hyperparameters.
I chose to tune the PID parameters by hand juding by look and feel of the car's driving pattern. 
I took an iterative approach, starting with the values from lecture. 
First I tuned the P portion as it is responsible for the biggest chunk of the response. 
I started with 0.3 and observed major ossciallations even when driving on the straight segments. So it was decreased successively till the car was able to drive on the straight segments remaining within the bounds of lane markings. 

After this differntial component, was tuned. I started with the value of 0.5 and kept increasing it till the overshoot kept decreasing. Once the overshoot started increasing, I left it to its old value.

Lastly, the integral component was tuned. In fact, the car drove perfectly even with zero value of this component. It seems simulator does not simulate the systematic bias.

in summary it was a twiddle algorithm implemented manually where I tuned each parameter individually on basis of look and feel of the cars driving before moving on to the next parameter. Here are the final value of parameters that I reached.

## Final Parameters

| Parameter|Value | 
|----------|------|
| P        |  0.15|
| I        |0.0003|
| D        |   2.6|

 
