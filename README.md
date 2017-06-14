# PID Controller Project
[Udacity - Self-Driving Car NanoDegree PID Controller Project]
(https://github.com/udacity/CarND-PID-Control-Project)


---

## Overview
In this project, the aim is to implement a PID controller in C++. The controller is responsible for calculating the steering angle. In addition, a second controller is responsible for calculating the throttle value.

## The Project
Applied two PID controllers, one for steering, and one for speed. In addition, the target speed is calculated as a function of steering angle and cte. The function has three additional parameters.

Nine parameters in total, implemented twiddle to find the parameters that let the controller drive with the highest mean speed.

PID class and Twiddle class are implemented for the PID Controller project.

The PID class has three methods.
* Init: initializes the PID controller, setting error values to zero.
* UpdateError: Calculates p_error, d_error, and i_error.
* TotalError: Calculates the total error value. Either the steering value or the throttle value.

The Twiddle class is responsible for applying the twiddle algorithm on the simulator. For each parameter set, it restarts the simulator. There are nine parameters to optimize.
* Kp, Kd, Ki for steering PID controller
* Kp, Kd, Ki for speed PID controller
* Min target speed
* Speed coefficient
* cte/steering limit for max speed

The target speed was calculated as follows:
cte_str value was calculated first. It takes the maximum of cte value or the steering value times 10. If the cte_str value was less than the cte/steering limit parameter, then the target speed was set to 100. Otherwise, the target speed was the sum of the min target speed parameter and the product of the speed coefficient and the (4.5 - cte_str) value.

## Twiddle
In every run, a parameter was changed by adding the dp * kp value for the parameter. Since, the parameters have different scales, a single dp value for each parameter was not suffice. Kd parameter changed more than the Kp parameter, and the Kp parameter changed more than the Kd parameter.

At the end of the run, if the car was able to complete the track, in other words, able to drive for the given size of steps, the mean cte error value and the mean steering value were compared with the lowest error and steering values. If they are both less than the previous best values, the parameters were selected as new best parameters. Only the Kp, Kd and Ki parameters for the steering PID controller were allowed to change by this condition. Since the other parameters may cause slowing the car.

In addition, if the car completed the track, and it had a higher mean speed, the parameters were accepted as newly selected parameters.

If the car was not able to complete the track, then it was needed to go further steps to select the new parameters.

The twiddle algorithm has run for several times. After each run, the best parameters were set to start with a new run. In addition, it was required to decrease the Kp values to find better parameters. The final values are stored in the Twiddle.cpp file.


Sample project video

[![PID Controller](http://img.youtube.com/vi/ATfoDwHBfaA/0.jpg)](http://www.youtube.com/watch?v=ATfoDwHBfaA)