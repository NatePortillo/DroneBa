#include "PID.h"

void PID_Init(PIDController *pid, float kp, float ki, float kd, float setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
    pid->setpoint = setpoint;
}

float PID_Compute(PIDController *pid, float measured_value) {
    float error = pid->setpoint - measured_value;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}