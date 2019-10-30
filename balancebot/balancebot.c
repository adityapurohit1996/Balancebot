/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
*
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <ncurses.h>
#include <unistd.h>

#include "balancebot.h"

#define CONTROLLER_SWITCH_ANGLE 15*3.14/180
#define max(a,b) (((a)>(b)) ? (a):(b))
#define min(a,b) (((a)<(b)) ? (a):(b))

/*******************************************************************************
* int main()
*
*******************************************************************************/
int main(int argc, char *argv[]){
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }
	
	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't botherrc_encoder_eqep_read(1
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);

	printf("starting gain thread... \n");
	pthread_t  gain_thread;
	initscr();
    cbreak();
	nodelay(stdscr, TRUE);
	rc_pthread_create(&gain_thread, set_gains, (void*) NULL, SCHED_OTHER, 0);

	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	rc_nanosleep(5E9); // wait for imu to stabilize
	mb_setpoints.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);
	pthread_mutex_init(&gains_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init(&mb_controls,&mb_setpoints);
	printf("kd2 = %f",mb_controls.kd_2);
	printf("initializing motors...\n");
	mb_motor_init();

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry.gyro_yaw_last = mb_state.gyro_yaw;
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	printf("starting gain thread... \n");
	// pthread_t  gain_thread;
	// initscr();
    // cbreak();
	// nodelay(stdscr, TRUE);
	// rc_pthread_create(&gain_thread, set_gains, (void*) NULL, SCHED_OTHER, 0);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}

	//save current gains
	FILE* fp;
	fp = fopen(CFG_PATH,"w");
	if (fp != NULL)
	{ 
		fprintf(fp,"%f %f %f %f %f %f\n",mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.F1,mb_controls.gyro_offset,mb_controls.left_motor_offset);
		fprintf(fp,"%f %f %f %f\n",mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.F2);
		fprintf(fp,"1.0 0.0 0.0 1\n");
		fprintf(fp,"1.0 0.0 0.0 1\n");
	}
	fclose(fp);

	// exit cleanly
	endwin();
	rc_mpu_power_off();
	mb_motor_cleanup(&mb_controls);
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
*
*
*******************************************************************************/



void balancebot_controller(){


	//lock state mutex
	pthread_mutex_lock(&gains_mutex);
	pthread_mutex_lock(&state_mutex);
	// Read IMU

	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X] ;


	// Read encoders and update odometry
	mb_odometry_update(&mb_odometry, &mb_state);
	mb_state.phi_L = (float)(mb_state.left_encoder)*2*3.14/(ENCODER_RES*GEAR_RATIO);
	mb_state.phi_R = (float)(mb_state.right_encoder)*2*3.14/(ENCODER_RES*GEAR_RATIO);


  // Calculate controller outputs
	mb_controller_update(&mb_controls,&mb_state,&mb_setpoints);
	
  if(!mb_setpoints.manual_ctl){
	  if(mb_state.d1_u > 0)
	  {
		mb_motor_set(RIGHT_MOTOR, maximum(-mb_state.d2_u_R,-0.999));
		mb_motor_set(LEFT_MOTOR, maximum(-mb_state.d2_u_L,-0.999));
	  }
	  else
	  {
		mb_motor_set(RIGHT_MOTOR, minimum(-mb_state.d2_u_R,0.999));
		mb_motor_set(LEFT_MOTOR, minimum(-mb_state.d2_u_L,0.999));
	  }
		
  }

  if(mb_setpoints.manual_ctl){
	  mb_motor_set(RIGHT_MOTOR, 0);
	  mb_motor_set(LEFT_MOTOR, 0);
    //send motor commands
  }
	/*
	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	*/

   	//unlock state mutex
	pthread_mutex_unlock(&gains_mutex);
    pthread_mutex_unlock(&state_mutex);

}




/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){
	    double drive_stick, turn_stick, input_mode; // input sticks
        // int i, ch, chan, stdin_timeout = 0; // for stdin input
        // char in_str[11];

		// while(rc_get_state()!=EXITING){
		// // clear out input of old data before waiting for new data
		// if(m_setpoints.man == STDIN) fseek(stdin,0,SEEK_END);
		// // sleep at beginning of loop so we can use the 'continue' statement
		// rc_usleep(1000000/SETPOINT_MANAGER_HZ);
		// // nothing to do if paused, go back to beginning of loop
		// if(rc_get_state() != RUNNING || m_input_mode == NONE) continue;
		// }

	while(1){

		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.
			turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
			drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;
			input_mode = rc_dsm_ch_normalized(DSM_CHOOSE_MODE)* DSM_DRIVE_POL;

			printf("%f %f %f\n",turn_stick, drive_stick, input_mode);
			if (input_mode) 
				mb_setpoints.manual_ctl = 1;
			else
				mb_setpoints.manual_ctl = 0;
		
		}

	 	rc_nanosleep(1E9 / 5);//RC_CTL_HZ
	} 
	return NULL;
}




/*******************************************************************************
* printf_loop()
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		// if(new_state==RUNNING){
		// 	printf("\nRUNNING: Hold upright to balance.\n");
		// 	printf("                 SENSORS               |            MOCAP            |");
		// 	printf("\n");
		// 	printf("    θ    |");
		// 	printf("    φ    |");
		// 	printf("theta_dot |");
		// 	printf("phi_dot   |");
		// 	printf("u         |");
		// 	printf("  L Enc  |");
		// 	printf("  R Enc  |");
		// 	printf("    X    |");
		// 	printf("    Y    |");
		// 	printf("    ψ    |");

		// 	printf("\n");
		// }
		
		// else if(new_state==PAUSED && last_state!=PAUSED){
		// 	printf("\nPAUSED\n");
		// }
		
		last_state = new_state;
		
		// if(new_state == RUNNING){
		 	printf("\r");
		// 	//Add Print stattements here, do not follow with /n
		// 	pthread_mutex_lock(&state_mutex);
		 	printf("theta = %7.3f  |", mb_state.theta);
		 	printf("phi_L = %7.3f  |", mb_state.phi_L);
			 printf("phi_R = %7.3f  |\n", mb_state.phi_R);
		// 	// printf("%7.3f  |", mb_state.theta_dot);
		// 	// printf("%7.3f  |", mb_state.phi_dot);
		// 	//printf("%7d  |", mb_state.left_encoder);
		// 	//printf("%7d  |", mb_state.right_encoder);
		// 	// printf("%7.3f  |", mb_state.opti_x);
		// 	// printf("%7.3f  |", mb_state.opti_y);
		// 	// printf("%7.3f  |", mb_state.opti_yaw);
		// 	//printf("%7.3f  |", mb_odometry.x);
		// 	//printf("%7.3f  |", mb_odometry.y);
		// 	//printf("%7.3f  |", mb_odometry.psi);
		// 	pthread_mutex_unlock(&state_mutex);
		// 	fflush(stdout);
		// }
		 rc_nanosleep(4E9/PRINTF_HZ);
	}
	return NULL;
}

void* set_gains(void* ptr) {
	while(rc_get_state()!=EXITING) {
		pthread_mutex_lock(&gains_mutex);
		//int ch = getch();
		mb_controller_load_config(&mb_controls);
		/*
		if (ch == ERR) 
			printf("no input\n");
		else {
			switch (ch) {
				case '1':
					mb_controls.kp_1 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case '2':
					mb_controls.kp_1 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case '3':
					mb_controls.ki_1 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case '4':
					mb_controls.ki_1 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case '5':
					mb_controls.kd_1 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case '6':
					mb_controls.kd_1 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'q':
					mb_controls.kp_2 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'w':
					mb_controls.kp_2 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'e':
					mb_controls.ki_2 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'r':
					mb_controls.ki_2 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 't':
					mb_controls.kd_2 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'y':
					mb_controls.kd_2 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'a':
					mb_controls.F1 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 's':
					mb_controls.F1 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'd':
					mb_controls.F2 += mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'f':
					mb_controls.F2 -= mb_controls.incre;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'z':
					mb_controls.incre += 0.01;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				case 'x':
					mb_controls.incre -= 0.01;
					printf("\n%f %f %f %f %f %f %f\n", mb_controls.kp_1, mb_controls.ki_1, mb_controls.kd_1, mb_controls.kp_2, mb_controls.ki_2, mb_controls.kd_2, mb_controls.incre);
					break;
				default:
					printf("%c\n",ch);
					break;
			}
		}
		*/
		pthread_mutex_unlock(&gains_mutex);
		rc_nanosleep(1E10/5);
	}
	return NULL;
}
