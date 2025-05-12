#ifndef PID_H // include guard
#define PID_H

#include <arduino.h>

class PID {
  private:
    //PID  coefficients
    float k_p, k_i, k_d;
    
    float previous_error;
    float current_error;
    float integral;

    float output;
    //bool is_output_limited;

    float min_output;
    float max_output;

    float min_integral;
    float max_integral;

  public:
    PID(float _k_p, float _k_i, float _k_d, float _min_output, float _max_output, float _min_integral, float _max_integral);
    float update(float _target_value, float _current_value, float _dt);
    
};

#endif /* PID_H */
