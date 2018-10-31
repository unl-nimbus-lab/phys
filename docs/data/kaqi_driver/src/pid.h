#ifndef _PID_H
#define _PID_H

struct PidGain
{
    float p_gain;
    float i_gain;
    float d_gain;
    float i_max;
    float i_min;
};

struct PidData
{
    PidGain gains;
    float p_error_last;
    float p_error;
    float i_error;
    float d_error;
    float command;
};

void deInitPid( struct PidData * pid );
void initPid( struct PidData * pid, float p, float i, float d, float i_max, float i_min );
void computeCommand( struct PidData * pid, float error, float dt );

#endif
