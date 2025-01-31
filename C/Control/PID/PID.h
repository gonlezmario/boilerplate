#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct
{
    int16_t Kp;
    int16_t Ki;
    int16_t Kd;
    int16_t dt;
    int16_t max_output;
    int16_t min_output;
    int16_t integral;
    int16_t prev_error;
    int16_t prev_measured;
    int16_t output;
} PIDController;

void PID_Init(PIDController *pid, int16_t kp, int16_t ki, int16_t kd, int16_t dt, int16_t max_output, int16_t min_output);
int16_t PID_Compute(PIDController *pid, int16_t setpoint, int16_t measured_value);
void PID_Reset(PIDController *pid);
#endif
