#ifndef PID
#define PID

typedef struct{
  float Kp;
  float Ki;
  float Kd;

  float integral;
  float previous_error;

  float output;
  float output_min;
  float output_max;
} PID_struct;

void PID_init(PID_struct *pid,float Kp,float Ki,float Kd,float output_min,float ouput_max);
float PID_update(PID_struct *pid,float setpoint,float measurement,float dt);

#endif
