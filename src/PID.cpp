#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->p = new double[3];
	this->p[0] = Kp;
	this->p[1] = Ki;
	this->p[2] = Kd;

	p_error = 0.0;
	d_error = 0.0;
	i_error = 0.0;
}

void PID::UpdateError(double cte) {
	i_error += cte;
	d_error = cte - p_error;
	p_error = cte;
}

double PID::TotalError() {
	return -p[0] * p_error - p[1] * i_error - p[2] * d_error;
}

