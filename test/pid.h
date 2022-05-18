#ifndef PID_H
#define PID_H
#include <stdlib.h> // for size_t
#include <stdint.h>

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

    int32_t max_out;
    int32_t max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], int32_t max_out, int32_t max_iout);
extern int32_t PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);
#endif
