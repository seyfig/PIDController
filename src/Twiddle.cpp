#include "Twiddle.h"
#include <iostream>

using namespace std;
#include <math.h>

// Number of parameters
const int n_param = 9;
// Number of parameters to change in Twiddle
const int t_param = 9;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init() {
	// True: Optimize parameters
	// False: Debug parameters
	this->twiddle = false;
	// Max sum of dp
	this->tolerance = 0.001;
	// Number of steps
	this->size = 1000;
	// Number of iterations, (1.0, 2.0)
	// if 2.0 only the errors in the second iteration are compared
	// if twiddle = false, size_mul should 1.0
	this->size_mul = 2.0;
	if (!this->twiddle) {
		this->size_mul = 1.0;
	}
	// If cte is greater than the limit, simulator reset
	// Parameters are not successful to complete the run
	this->cte_limit = 4.5;

	this->p = new double[n_param];
	// Kp for PID steering
	this->p[0] = 0.04;
	// Ki for PID steering
	this->p[1] = 0.00777101;
	// Kd for PID steering
	this->p[2] = 1.4;
	// Min Target Speed
	this->p[3] = 57.4;
	// Speed over cte/steering coefficient
	this->p[4] = 9.511;
	// Kp for PID speed
	this->p[5] = 0.2665;
	// Ki for PID speed
	this->p[6] = 0.00016;
	// Kd for PID speed
	this->p[7] = 0.0026;
	// cte/steering limit for max speed
	this->p[8] = 1.1355;



	this->dp = new double[n_param];
	this->dp[0] = 1.0;
	this->dp[1] = 1.0;
	this->dp[2] = 1.0;
	this->dp[3] = 1.0;
	this->dp[4] = 1.0;
	this->dp[5] = 1.0;
	this->dp[6] = 1.0;
	this->dp[7] = 1.0;
	this->dp[8] = 1.0;

	// Multipliers for each parameter
	this->kp = new double[n_param];
	/*this->kp[0] = 0.02;
	this->kp[1] = 0.002;
	this->kp[2] = 0.5;
	this->kp[3] = 5.0;
	this->kp[4] = 1.0;
	this->kp[5] = 0.05;
	this->kp[6] = 0.0001;
	this->kp[7] = 0.001;
	this->kp[8] = 0.5;*/

	this->kp[0] = 0.01;
	this->kp[1] = 0.001;
	this->kp[2] = 0.1;
	this->kp[3] = 2.0;
	this->kp[4] = 0.5;
	this->kp[5] = 0.02;
	this->kp[6] = 0.0001;
	this->kp[7] = 0.001;
	this->kp[8] = 0.2;

	this->pidx = -1;
	this->param_second = false;
	this->reset = false;
	this->best_counter = 0;
	this->best_error = 1000000.0;
	this->best_steering = 1000000.0;
	this->best_speed = 0.0;
	this->Restart();
}
void Twiddle::Restart() {
	// PID steering
	this->pid.Init(this->p[0],this->p[1],this->p[2]);
	// PID speed
	this->pids.Init(this->p[5],this->p[6],this->p[7]);
	this->counter = 0;
	this->total_error = 0.0;
	this->total_steering = 0.0;
	this->total_speed = 0.0;
	this->total_target_speed = 0.0;
	this->throttle_sum = 0.0;
	this->max_cte = 0.0;
	this->max_steering = 0.0;
}
void Twiddle::Next() {
	// Average total squared error
	double error = (this->size_mul * this->total_error / this->counter);
	// Average total squared steering
	double steering = (this->size_mul * this->total_steering / this->counter);
	// Average speed
	double speed = (this->total_speed / this->counter);
	double target_speed = (this->total_target_speed / this->counter);
	// If it is the initial run
	if (this->pidx == -1) {
		this->best_error = error;
		this->best_steering = steering;
		this->best_speed = speed;
		this->best_counter = counter;
		this->pidx = 0;

		std::cout<<" Init:Init:"<<best_error<<"; STR: "<<steering<<"; SPD: "<<speed<<"; pidx:"<<pidx;
		this->PrintAverage();
		this->Printp();
		this->Printdp();
		std::cout<<std::endl;
	}
	else {
		bool is_best = false;
		// if track completed
		if (this->counter == this->size_mul * this->size) {
			// When both error and steering values are better than the previous best,
			// AND
			// If the parameters go further than the previous best or
			// only the steering PID parameters are optimized
			// Decreasing speed to get less error does not count.
			if ((error < this->best_error && steering < this->best_steering)
				&& (this->pidx < 3 || this->counter > this->best_counter)) {
				this->best_error = error;
				this->best_steering = steering;
				if (speed > best_speed  || this->counter > this->best_counter) {
					this->best_speed = speed;
				}
				this->best_counter = this->counter;
				is_best = true;
				std::cout<<" E&S ";
			}
			else if (speed > best_speed) {
				is_best = true;
				best_speed = speed;
				std::cout<<" FST ";
			}
			// Only for log purpose
			else if (error < this->best_error) {
				std::cout<<" ERR ";
			}
			// Only for log purpose
			else if (steering < best_steering) {
				std::cout<<" STE ";
			}
			// Only for log purpose
			else {
				std::cout<<" CMP ";
			}
		}
		else {
			// When track is not completed, error is calculate in terms of the number of steps
			// less error means, car went further
			if (error < this->best_error) {
				is_best = true;
				this->best_counter = this->counter;
				this->best_error = error;
				this->best_steering = steering;
				this->best_speed = speed;
				std::cout<<" FAR ";
			}
			else {
				std::cout<<" INC ";
			}
		}
		// Twiddle better results
		if (is_best) {
			dp[pidx] *= 1.1;

			std::cout<<"New:"<<error<<"; STR: "<<steering<<"; SPD: "<<speed<<"; pidx:"<<pidx;
			this->PrintAverage();
			this->Printp();
			this->Printdp();
			std::cout<<std::endl;
			this->param_second = false;
			this->pidx += 1;
		}
		// Twiddle NO better results
		else {
			// If second try for the parameter,
			// Change did not help, decrease dp
			if (this->param_second) {
				std::cout<<"NOCH:"<<error<<"; STR: "<<steering<<"; SPD: "<<speed<<"; pidx:"<<pidx;
				this->PrintAverage();
				this->Printp();
				this->p[pidx] += (this->dp[pidx] * this->kp[pidx]);
				dp[pidx] *= 0.9;
				this->Printdp();
				std::cout<<std::endl;
				param_second = false;
				pidx += 1;
			}
			// Increasing parameter did not help, try decreasing
			else {
				std::cout<<"Rtry:"<<error<<"; STR: "<<steering<<"; SPD: "<<speed<<"; pidx:"<<pidx;
				this->PrintAverage();
				this->Printp();
				this->p[pidx] -= (2.0 * this->dp[pidx] * this->kp[pidx]);
				param_second = true;
				this->Printdp();
				std::cout<<std::endl;
			}
		}
	}
	// Change next parameter
	this->pidx %= t_param;
	if (this->pidx == 0) {
		double dp_sum = 0.0;
		for (int i=0;i < t_param; ++i) {
			dp_sum += this->dp[i];
		}
		if (dp_sum <= this->tolerance) {
			this->twiddle = false;
			std::cout<<"BEST: ";
			this->Printp();
			this->Printdp();
		}
	}
	if (this->twiddle) {

		if (!this->param_second) {
			this->p[this->pidx] += (this->dp[pidx] * this->kp[pidx]);
		}
		this->Restart();
		this->reset = true;
	}

}

void Twiddle::UpdateError(double cte, double speed) {
	bool debug = false;
	this->pid.UpdateError(cte);
	this->pid_steering = this->pid.TotalError();

	double cte_speed = 0.0;
	double target_speed = 0.0;

	double cte_abs = fabs(cte);
	double str_abs = fabs(pid_steering);

	// Constant speed
	//cte_speed = speed - this->p[6];
	// Speed depends on steering and cte
	double cte_str = std::max(cte_abs, std::min(0.4364, str_abs) * 10.0);
	// Speed only depends steering
	//double cte_str = std::min(0.4364, fabs(pid_steering)) * 10.0;

	// if cte_str is smaller than cte/steering limit, set target speed to maximum
	if (cte_str <= this->p[8]) {
		target_speed = 100.0;
	}
	else {
		target_speed = this->p[3] + this->p[4] * (4.5 - cte_str);
	}

	cte_speed = speed - target_speed;
	this->pids.UpdateError(cte_speed);
	this->pid_throttle = this->pids.TotalError();


	if (pid_throttle > 1.0) {
		pid_throttle = 1.0;
	}
	else if (pid_throttle < -1.0) {
		pid_throttle = -1.0;
	}
	this->throttle_sum += pid_throttle;
	this->counter++;

	if (this->size_mul > 1.0) {
		if (this->counter >= (this->size)) {
			this->total_error += cte * cte;
			this->total_steering += (this->pid_steering * this->pid_steering);
		}
	}
	else {
		this->total_error += cte * cte;
		this->total_steering += (this->pid_steering * this->pid_steering);
	}
	this->total_speed += speed;
	this->total_target_speed += target_speed;
	if (cte_abs > this->max_cte) {
		this->max_cte = cte_abs;
	}
	if (str_abs > this->max_steering) {
		this->max_steering = str_abs;
	}

	// If in twiddle optimization
	if (this->twiddle) {
		// If Kp is negative, stop run
		if (this->p[0] < 0) {
			std::cout<<"-Kp: "<<this->counter<<"; ";
			total_error = 1000000.0;
			total_steering = 1000000.0;
			this->Next();
		}
		// If cte is greater than the limit, car is out of the road,
		// stop run
		else if (fabs(cte) >= this->cte_limit) {
			std::cout<<"CTE: "<<this->counter<<"; ";
			total_error = 1000000.0;
			total_steering = 1000000.0;
			this->Next();
		}
		// If car is too slow or going backwards
		// TODO: do not cover all scenarios
		else if (this->counter > 50 &&
				 (this->throttle_sum < 0.0 ||
				 	this->total_speed / this->counter < 5.0)) {
			std::cout<<"SPD: "<<this->counter<<"; ";
			total_error = 1000000.0;
			total_steering = 1000000.0;
			this->Next();
		}
		// Run is completed
		else if (this->counter == this->size_mul * this->size) {
			std::cout<<"CMP: "<<this->counter<<"; ";
			this->Next();
		}
	}
	// If not in twiddle optimization
	else {
		std::cout<<this->counter;
		std::cout<<". MEAN CTE:"<<(this->total_error/ this->counter);
		std::cout<<"; MEAN SPEED:"<<(this->total_speed/this->counter);
		std::cout<<"; CURRENT CTE:"<<cte<<"; CURRENT SPEED:"<<speed<<"; STEERING:"<<this->pid_steering;
		std::cout<<"; THROTTLE:"<<this->pid_throttle;
		std::cout<<"; MAX CTE:"<<this->max_cte<<std::endl;
	}
}
void Twiddle::PrintAverage() {
	std::cout<<";max_cte:"<<this->max_cte<<";max_str:"<<this->max_steering;
	std::cout<<";avg_trg:"<<this->total_target_speed/this->counter;
}
void Twiddle::Printp() {
	std::cout<<"; p:";
	for (int i = 0; i < t_param - 1; ++i){
		std::cout<<p[i]<<";";
	}
	std::cout<<p[t_param - 1];
}

void Twiddle::Printdp() {
	std::cout<<"; dp:";
	for (int i = 0; i < t_param - 1; ++i){
		std::cout<<dp[i]<<";";
	}
	std::cout<<dp[t_param - 1];
}

