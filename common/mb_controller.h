#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "mb_structs.h"
#define CFG_PATH                    "pid.cfg"
#define M_PI                        3.14159265358979323846

int mb_controller_init();
int mb_controller_load_config();
int mb_controller_update(mb_controls_t* mb_controls, mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_controller_cleanup();
float maximum(float a, float b);
float minimum(float a, float b);
float gamma_diff(float set_gamma, float gamma);
int traj_planner(float x, float y, float psi, mb_odometry_t** traj);
int traj_planner_line(float x, float vel, float acc_1, float acc_2, mb_odometry_t** traj);
#endif
