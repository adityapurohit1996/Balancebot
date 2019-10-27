#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

#include <rc/math/filter.h>

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   gyro_yaw;
    float   phi;               // average wheel angle (rad)
    float theta_dot;
    float phi_dot;
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    double d1_u;
    double d2_u;
    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float theta;
    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int  manual_ctl;
};

typedef struct mb_controls mb_controls_t;
struct mb_controls{

    rc_filter_t D1;
    rc_filter_t D2;

    float kp_1;
    float ki_1;
    float kd_1;
    float F1 ;
    float kp_2 ;
    float ki_2;
    float kd_2 ;
    float F2 ;

};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
    float gyro_yaw_last;    //for gyrodometry
};

#endif
