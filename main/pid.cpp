#include "pid.h"

//void PID_update(PID_struct *pid, float, float setpoint, float measurement, float dt);

void PID_init(PID_struct *pid,float Kp,float Ki,float Kd,float output_min,float output_max){
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->output = 0.0f;
  pid->output_min = output_min;
  pid->output_max = output_max;
}

float PID_update(PID_struct *pid, float setpoint, float measurement, float dt){
  float error = setpoint - measurement;
  float P = pid->Kp * error;
  float new_integral = pid->integral + (error * dt);
  float I = pid->Ki * new_integral;
  float der = (error - pid->previous_error)/dt;
  float D = pid->Kd * der;
  float output = P+I+D;

  if (output <= pid->output_max &&
      output >= pid->output_min)
  {
      pid->integral = new_integral;
  }

  if (output > pid->output_max)
  {
      output = pid->output_max;
  }

  if (output < pid->output_min)
  {
      output = pid->output_min;
  }

  pid->output = output;
  pid->previous_error = error;
  return output;
}