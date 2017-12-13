#include "PID.h"
#include <iostream>
#include <numeric>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  cout << "Init..." << endl;
  cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
  
  // initialize coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  // initialize errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}


void PID::UpdateError(double cte) {
  //cout << "UpdateError..." << endl;
  
  //cout << "cte: " << cte << endl;
  
  double previous_cte = p_error;
  
  d_error = cte - previous_cte;
  p_error = cte;
  i_error += cte;
  
  //cout << "p_error: " << p_error << " i_error: " << i_error << " d_error: " << d_error << endl;
}


double PID::TotalError() {
  //cout << "TotalError..." << endl;
  
  double total_error;
  total_error = -Kp * p_error - Ki * i_error - Kd * d_error;
  
  //cout << "TotalError: " << total_error << endl;
  
  return total_error;
}

