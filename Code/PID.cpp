#include "PID.h"

PID::PID(float _k_p, float _k_i, float _k_d, float _min_output, float _max_output, float _min_integral, float _max_integral): 
          k_p(_k_p), 
          k_i(_k_i), 
          k_d(_k_d), 
          min_output(_min_output), 
          max_output(_max_output),
          min_integral(_min_integral), 
          max_integral(_max_integral){}

float PID::update(float _target_value, float _current_value, float _dt) {

  current_error = _target_value - _current_value;

  integral += current_error * _dt;

  //simple limitation of integral
  if (integral > max_integral) {
    integral = max_integral;
  }
  if (integral < min_integral) {
    integral = min_integral;
  }
  
  output = current_error * k_p + integral * k_i + ((current_error - previous_error) / _dt) * k_d;

  //is_output_limited = false;
  
  if (output > max_output) {
    output = max_output;
    //is_output_limited = true;
  }
  if (output < min_output) {
    output = min_output;
    //is_output_limited = true;
  }

  /*
  //TODO: integral clamping
  //to clamp or not to clamp
  if (is_output_limited && ((current_error >= 0 && output >= 0) || (current_error < 0 && output < 0))) {
    
  }
  */
  
  previous_error = current_error;

  return output;
}
