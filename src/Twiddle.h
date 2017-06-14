#ifndef Twiddle_H
#define Twiddle_H

#include "PID.h"

class Twiddle {
public:

  /*
  * Coefficient Changes
  */
  double *p;
  double *dp;
  double *kp;

  int counter;
  int best_counter;
  double pid_steering;
  double pid_throttle;
  double total_error;
  double total_steering;
  double total_speed;
  double best_error;
  double best_steering;
  double best_speed;
  double throttle_sum;
  double max_cte;
  double max_steering;
  double total_target_speed;
  double avg_target_speed;


  int size;
  double size_mul;

  // Parameter index 0 > Kp, 1 > Ki, 2 > Kd
  int pidx;
  bool param_second;

  double tolerance;
  bool twiddle;
  bool reset;
  double cte_limit;

  /*
  PID Steering
  */
  PID pid;
  /*
  PID Speed
  */
  PID pids;

  /*
  * Constructor
  */
  Twiddle();

  /*
  * Destructor.
  */
  virtual ~Twiddle();


  void Init();
  void Restart();
  void Next();
  void UpdateError(double cte, double speed);
  void Printp();
  void Printdp();
  void PrintAverage();
};

#endif /* PID_H */
