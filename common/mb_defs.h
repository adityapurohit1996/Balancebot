/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc.
*      and your specific configuration...
*
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR                  1 // id of left motor
#define RIGHT_MOTOR                 2 // id of right motor
#define MDIR1_CHIP                  1 // chip of MDIR1 gpio pin
#define MDIR1_PIN                   28 //MDIRR1 gpio(CHIP.PIN) P9.12
#define MDIR2_CHIP                  1// chip of MDIR2 gpio pin
#define MDIR2_PIN                   16 //  MDIRR2 gpio(CHIP.PIN) P9.15
#define MOT_BRAKE_EN               0,20  // gpio0.20  P9.41
#define MOT_1_POL                   1   // polarity of motor 1
#define MOT_2_POL                   0 // polarity of motor 2
#define ENC_1_POL                   -1 // polarity of encoder 1
#define ENC_2_POL                   1 // polarity of encoder 2
#define MOT_1_CS                    0 // analog in of motor 1 current sense
#define MOT_2_CS                    1 // analog in of motor 2 current sense
#define GEAR_RATIO                 20.4 // gear ratio of motor
#define ENCODER_RES                 48 //979.62 // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER              0.084 // diameter of wheel in meters
#define WHEEL_BASE                  0.208 // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY       0.1 // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY      0.1 // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ            100 // main filter and control loop speed
#define DT                       0.01 // 1/sample_rate
#define PRINTF_HZ                  10 // rate of print loop
#define RC_CTL_HZ                  25 // rate of RC data update
#define MPWM1_PIN                  'A' // PWM pin for first motor
#define MPWM2_PIN                  'B' //PWM pin for second motor
#define PWM_SUBSYSTEM               1 // PWM subsystem
#define THRESHOLD_ANGLE             1*3.14/180 // Threshold angle for I controller

//for controllers
#define SOFT_START_SEC		0.7
// inner loop controller 100hz
#define D1_ORDER		3
#define D1_NUM			{-1372}
#define D1_DEN			{ 1.000, 112.8, -128, 4321}
#define D1_NUM_LEN		1
#define D1_DEN_LEN		4
#define D1_SATURATION_TIMEOUT	0.4

// outer loop controller 100hz
#define	D2_ORDER		2
#define D2_NUM			{-2.815, 0, 146.2}
#define D2_DEN			{1.00000,  0, 0}
#define D2_NUM_LEN		3
#define D2_DEN_LEN		3
#define THETA_REF_MAX		0.33

#endif
