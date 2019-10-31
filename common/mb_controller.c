#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(mb_controls_t* mb_controls, mb_setpoints_t* mb_setpoints){
    mb_controller_load_config(mb_controls);
    /* TODO initialize your controllers here*/
    mb_setpoints->theta = mb_controls->gyro_offset;
    mb_setpoints->phi = 0;
    mb_setpoints->gamma = 0;
    mb_setpoints->manual_ctl = 0;
	mb_setpoints->fwd_velocity = 0;
    /*
    mb_controls->D1 = RC_FILTER_INITIALIZER;
    mb_controls->D2 = RC_FILTER_INITIALIZER;
    double D1_num[] = D1_NUM;
    double D1_den[] = D1_DEN;
    double D2_num[] = D2_NUM;
    double D2_den[] = D2_DEN;
    */
	rc_filter_pid(&mb_controls->D1, mb_controls->kp_1, 0, mb_controls->kd_1, 1/mb_controls->F1, DT);
    rc_filter_pid(&mb_controls->Di, 0, mb_controls->ki_1, 0, 1/mb_controls->F1, DT);
	rc_filter_pid(&mb_controls->D2, mb_controls->kp_2, mb_controls->ki_2, mb_controls->kd_2, 1/mb_controls->F2, DT);
	rc_filter_pid(&mb_controls->D3, mb_controls->kp_3, mb_controls->ki_3, mb_controls->kd_3, 1/mb_controls->F3, DT);


    rc_filter_enable_saturation(&mb_controls->D1, -1.0, 1.0);
    rc_filter_enable_soft_start(&mb_controls->D1, SOFT_START_SEC);
    rc_filter_enable_saturation(&mb_controls->Di, -0.7, 0.7);
    rc_filter_enable_soft_start(&mb_controls->Di, SOFT_START_SEC);

    rc_filter_enable_saturation(&mb_controls->D2, -THETA_REF_MAX, THETA_REF_MAX);
    rc_filter_enable_soft_start(&mb_controls->D2, SOFT_START_SEC);

    rc_filter_enable_saturation(&mb_controls->D3, -THETA_REF_MAX, THETA_REF_MAX);
    rc_filter_enable_soft_start(&mb_controls->D3, SOFT_START_SEC);
    mb_controls->incre = 0.01;

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*#define M_PI                        3.14159265358979323846
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(mb_controls_t* mb_controls){
    float temp[14];
    int i;
    FILE* fp = fopen(CFG_PATH, "r");
    if (fp == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    else
    {
    for (i=0;i<14;i++)
    {
        fscanf(fp,"%f",&temp[i]);	
    }
    mb_controls->kp_1 = -temp[0];
    mb_controls->ki_1 = -temp[1];
    mb_controls->kd_1 = -temp[2];
    mb_controls->F1 = temp[3];
    mb_controls->gyro_offset = temp[4];
    mb_controls->left_motor_offset = temp[5];
    mb_controls->kp_2 = temp[6];
    mb_controls->ki_2 = temp[7];
    mb_controls->kd_2 = temp[8];
    mb_controls->F2 = temp[9];
    mb_controls->kp_3 = temp[10];
    mb_controls->ki_3 = temp[11];
    mb_controls->kd_3 = temp[12];
    mb_controls->F3 = temp[13];
    fclose(fp);
	}
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
*
*
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_controls_t* mb_controls, mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){
    /*TODO: Write your controller here*/

    
 
        //if(fabs(mb_state.phi_dot) > 0.001) setpoint.phi += setpoint.phi_dot*DT;
    mb_state->d2_u = rc_filter_march(&mb_controls->D2,mb_setpoints->phi - mb_state->phi);
    mb_setpoints->theta = mb_state->d2_u + mb_controls->gyro_offset;

    static float theta_error;
    theta_error =  mb_setpoints->theta-mb_state->theta;
    mb_state->d1_u = rc_filter_march(&mb_controls->D1,theta_error);
    if(theta_error< -(THRESHOLD_ANGLE) || theta_error > THRESHOLD_ANGLE)
    {
        mb_state->d1_u += rc_filter_march(&mb_controls->Di,theta_error);
    }

     mb_state->u = mb_state->d1_u;

    //consider the roundoffs when calculateing the gamma difference
    float gamma_d = gamma_diff(mb_setpoints->gamma, mb_state->gamma);
    mb_state->d3_u = rc_filter_march(&mb_controls->D3,gamma_d);
    mb_state->left_cmd = mb_state->u - mb_state->d3_u;
    mb_state->right_cmd = mb_state->u + mb_state->d3_u;

    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
*
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(mb_controls_t* mb_controls){
    rc_filter_free(&mb_controls->D1);
    rc_filter_free(&mb_controls->Di);
    rc_filter_free(&mb_controls->D2);
    //rc_filter_free(&mb_controls->D3);
    return 0;
}

float maximum (float a, float b)
{
    return a > b ? a : b;
}

float minimum (float a, float b)
{
    return a < b ? a : b;
}

float gamma_diff(float set_gamma, float gamma) 
{
    if ((set_gamma<0) && (gamma>0)) 
    {
        if (fabs(set_gamma+2*M_PI-gamma) < fabs(set_gamma-gamma))
            return (set_gamma+2*M_PI-gamma);
    }
    if ((gamma<0) && (set_gamma>0)) 
    {
        if (fabs(set_gamma-gamma-2*M_PI) < fabs(set_gamma-gamma))
            return (set_gamma-gamma-2*M_PI);
    }
    return (set_gamma-gamma);
}