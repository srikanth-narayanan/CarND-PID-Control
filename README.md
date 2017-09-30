# PID - Controls

This project aims at building a PID controller. This controller will connect to the udacity simulator to provide steering angle and throttle to race the car around the track autonomously. The PID controller is tunned using the twiddle algorithm. The PID controller must send steering and throttle values to the car while ensuring to keep within the track limits and drive around the track.

## Key Discussion points

The P (proportional) component of the controller has direct influence on the car behaviour on the track. It steers in order to compensate for the cross track error (CTE), which is the lane center. If the car move towards the left it steer hard towards the right and vice versa to cause a steady oscillations.

[Video of P only control.](https://github.com/srikanth-narayanan/CarND-PID-Control/blob/master/Videos/P_Only_Control.mp4)

The D (differential) component of the controller counteracts the P compoenent tendency to overshoot the lane center and creates a critically damped system.

[Video of PD only control.](https://github.com/srikanth-narayanan/CarND-PID-Control/blob/master/Videos/PD_Control.mp4)

The I (integral) component of the controller compensates for the bias in the CTE which prevents the PD controller from the reaching the lane center. A steering drift or brahviour around the curves are the key areas the I component plays a key role.

[Video of PID control.](https://github.com/srikanth-narayanan/CarND-PID-Control/blob/master/Videos/PID_Controls.mp4)

### Hyperparameters Tuning

The gain values for kp, kd and ki are first manuallu tunned. This has to be done becasue the margin of errors from the track left a very narrow corridor for auotomatic tunning using twiddle algorithm. It was very easy for the car to leave the track. Once the base value for Kp, Ki and Kd was achieved for a fixed throttle value, 1/3rd of the track was used to start the initial tunning of the parameter using twiddle. Once a certain step point wass reached the simulator was reset and it used the tweaked PID values and the continued until no further improvements was observed.

The new value is then used to test the car run on the full course, so that it can negotiate the track without leaving the track. I was able to sucessfully get a good parameters, but the throttle was fixed. In order to increase the performance a PID control for the throttle was implemented. I ran the twiddle algorithm again for steering and throttle for the optimised value.

The final value of Kp, Ki and Kd for steering was 0.103731, 0.000219164, 3.0027 and throttle was 0.290158, 9.1e-07, 0.1 respectively.

[Video of the final tuned PID control for steering and throttle]

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
  * Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
