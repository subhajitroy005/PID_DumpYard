/*
 * PID.h
 *
 *  Created on: 7-Sept-2022
 *      Author: Venom
 */
#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "math.h"
#include "stdint.h"



#define __8BIT_OUTPUT_MAX    180 
#define __8BIT_OUTPUT_MIN    0

#define __MAXMEASUREMENT    1000 // Lets say for now 

typedef struct 
{
    /* PID Gains*/
    float kp;
    float ki;
    float kd;

    /* P I D terms */
    float propotional;
    float integrator;
    float derivative;

    /*Error term*/
    float prevErr; 

    /*Measurement*/
    float prevMeasure; //Instead of the "error" term, for anti derivative kick


    /*Lowpass Filter Tau for Diffrentiator*/
    float tau;

    /*Sample time */
    float Ts;

    /*Limits, to clamp the windup error and the system saturation
    * Since the computations are mapped for 0-100% 
    * Max possible limit must be within 0.0 and 1.0 
    * Anything above this is not acceptable. 
    */
    float limMin;
    float limMax;

    float limMinInt;
    float limMaxInt;


    /*And ofcourse, the output*/
    float pidout;

}PID_Handle_t;

//////////////////////////////////////////////////////API_CALLS///////////////////////////////////////
/*Initializer*/
uint8_t PID_Init(PID_Handle_t *pid);

void SetSampleRate(PID_Handle_t *pid, float sampleTime);

/*PID computation API*/
float PID_Compute(PID_Handle_t *pid, float measurement, float setPoint, float hal_tick);


#endif //endo of file