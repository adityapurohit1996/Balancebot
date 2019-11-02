/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality
*
*******************************************************************************/
#include "math.h"
#include "../balancebot/balancebot.h"
#include <rc/encoder_eqep.h>

#define M_PI                        3.14159265358979323846
#define ENCODER_RES                 48 // encoder counts per motor shaft revolution
#define GEAR_RATIO                  20.4 // gear ratio of motor
#define WHEEL_RADIUS                0.042      //radius of wheel in meters
#define WHEEL_BASE                  0.208 // wheel separation distance in meters
#define GYRODOM_THRESH              0.125*M_PI/180




void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float psi){
    /* TODO */
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = psi;
    // mb_state.left_encoder = -rc_encoder_eqep_read(1);//encoder 1 is reversed
    // mb_state.right_encoder = rc_encoder_eqep_read(2);

}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    /* TODO */
    int cur_enc_left = -rc_encoder_eqep_read(1);
    int cur_enc_right = rc_encoder_eqep_read(2);
    int delta_enc_left = cur_enc_left - mb_state->left_encoder;
    int delta_enc_right = cur_enc_right - mb_state->right_encoder;
    mb_state->left_encoder = cur_enc_left;
    mb_state->right_encoder = cur_enc_right;

    float delta_phi_left = (float)(2*M_PI*delta_enc_left)/ENCODER_RES/GEAR_RATIO;
    float delta_phi_right = (float)(2*M_PI*delta_enc_right)/ENCODER_RES/GEAR_RATIO;

    float delta_s_w_left = WHEEL_RADIUS*delta_phi_left;
    float delta_s_w_right = WHEEL_RADIUS*delta_phi_right;
    float delta_psi = (delta_s_w_right - delta_s_w_left)/WHEEL_BASE;
    float delta_d = (delta_s_w_right + delta_s_w_left)/2;

    float delta_x = delta_d*cos(mb_odometry->psi + delta_psi/2);
    float delta_y = delta_d*sin(mb_odometry->psi + delta_psi/2);

    //update odometry

    mb_odometry->x += delta_x;
    mb_odometry->y += delta_y;

    //gyrodometry
    float cur_gyro_yaw = mpu_data.dmp_TaitBryan[TB_YAW_Z];
    float delta_gyro_yaw = cur_gyro_yaw - mb_odometry->gyro_yaw_last;
    float delta_G_O = delta_gyro_yaw - delta_psi;
    if (fabs(delta_G_O) > GYRODOM_THRESH)
        mb_odometry->psi += delta_gyro_yaw;
    else
        mb_odometry->psi += delta_psi;
    //clamp odometry
    mb_odometry->psi = mb_clamp_radians(mb_odometry->psi);

    mb_odometry->gyro_yaw_last = cur_gyro_yaw;

}


float mb_clamp_radians(float angle){
    if (angle<-M_PI)
        angle += 2*M_PI;
    if (angle>M_PI)
        angle -= 2*M_PI;
    return angle;
}
