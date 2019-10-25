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
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    mb_setpoints->manual_ctl = false;
	mb_setpoints->fwd_velocity = 0;

    mb_controls->D1 = RC_FILTER_INITIALIZER;
    mb_controls->D2 = RC_FILTER_INITIALIZER;
    double D1_num[] = D1_NUM;
    double D1_den[] = D1_DEN;
    double D2_num[] = D2_NUM;
    double D2_den[] = D2_DEN;

	rc_filter_pid(&mb_controls->D1, mb_controls->kp_1, mb_controls->ki_1, mb_controls->kd_1, DT*4, DT);
	rc_filter_pid(&mb_controls->D2, mb_controls->kp_2, mb_controls->ki_2, mb_controls->kd_2, DT*4, DT);


    rc_filter_enable_saturation(&D1, -1.0, 1.0);
    rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

    rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
    rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/
    fclose(file);
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

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){
    /*TODO: Write your controller here*/
    if(fwd_velocity == 0){
        //if(fabs(mb_state.phi_dot) > 0.001) setpoint.phi += setpoint.phi_dot*DT;
        mb_state->d2_u = rc_filter_march(&D2,0-mb_state->phi);
        mb_setpoints->theta = mb_state->d2_u;
    }
    else mb_setpoints->theta = 0.0;

    mb_state->d1_u = rc_filter_march(&D1,(mb_setpoints->theta-mb_state->theta));
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
    rc_filter_free(&mb_controls->D2);
    rc_filter_free(&mb_controls->D3);
    return 0;
}
