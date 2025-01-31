#include "PID.h"

#define PID_SCALING_FACTOR 1000 // Scaling factor for fixed-point calculations

void PID_Init(PIDController *pid, int16_t kp, int16_t ki, int16_t kd, int16_t dt, int16_t max_output, int16_t min_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
    pid->max_output = max_output;
    pid->min_output = min_output;

    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measured = 0;
    pid->output = 0;
}

int16_t PID_Compute(PIDController *pid, int16_t setpoint, int16_t measured_value)
{
    int16_t error = setpoint - measured_value;

    // Proportional term
    int32_t Pout = (pid->Kp * (int32_t)error) / PID_SCALING_FACTOR;

    // Integral term (with anti-windup check)
    int32_t integral_temp = pid->integral + ((int32_t)error * pid->dt) / PID_SCALING_FACTOR;
    if (integral_temp > pid->max_output)
        integral_temp = pid->max_output;
    if (integral_temp < pid->min_output)
        integral_temp = pid->min_output;
    pid->integral = (int16_t)integral_temp;

    int32_t Iout = (pid->Ki * (int32_t)pid->integral) / PID_SCALING_FACTOR;

    // Derivative term (without using error to avoid derivative kick)
    int32_t derivative = -(measured_value - pid->prev_measured) / pid->dt;
    int32_t Dout = (pid->Kd * derivative) / PID_SCALING_FACTOR;

    pid->output = (int16_t)(Pout + Iout + Dout);

    if (pid->output > pid->max_output)
        pid->output = pid->max_output;
    if (pid->output < pid->min_output)
        pid->output = pid->min_output;

    pid->prev_error = error;
    pid->prev_measured = measured_value;

    return pid->output;
}

void PID_Reset(PIDController *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measured = 0;
    pid->output = 0;
}
