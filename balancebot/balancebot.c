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

	int c;
	if(!strcmp("dsm", argv[1])) {
		mb_setpoints.manual_ctl = 0;
	} 
	else {
		mb_setpoints.manual_ctl = 1;
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

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();

	printf("getting gains");
	mb_get_gains(&mb_gains);
	printf("%lf\t%lf\t%lf\t%lf\n",mb_gains.K1,mb_gains.K2,mb_gains.K3,mb_gains.K4);

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
	fp = fopen("gains.txt","w");
	if (fp != NULL)
	{ 
		fprintf(fp,"%f\n",mb_gains.K1);
		fprintf(fp,"%f\n",mb_gains.K2);
		fprintf(fp,"%f\n",mb_gains.K3);
		fprintf(fp,"%f\n",mb_gains.K4);
		fprintf(fp,"%f\n",mb_gains.Nbar);
		fprintf(fp,"%f\n",mb_gains.temp1);
		fprintf(fp,"%f\n",mb_gains.temp2);
		fprintf(fp,"%f\n",mb_gains.temp3);
	}
	fclose(fp);
	
	// exit cleanly
	// endwin();
	rc_mpu_power_off();
	mb_motor_cleanup();
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
	pthread_mutex_lock(&state_mutex);
	pthread_mutex_lock(&gains_mutex);
	// Read IMU

	static float last_theta=0,last_theta_2,last_theta_3,last_phi;
	mb_state.theta = (mpu_data.dmp_TaitBryan[TB_PITCH_X] + last_theta)/2;

	// Read encoders and update odometry 
	mb_odometry_update(&mb_odometry, &mb_state);

	mb_state.phi = (float)(mb_state.right_encoder + mb_state.left_encoder)*3.14/(ENCODER_RES*GEAR_RATIO);

	mb_state.theta_dot = (mb_state.theta - last_theta_3) * SAMPLE_RATE_HZ/3;
	mb_state.phi_dot = (mb_state.phi - last_phi) * SAMPLE_RATE_HZ;

	last_theta = mb_state.theta;
	last_theta_2 = last_theta;
	last_theta_3 = last_theta_2;
	last_phi = mb_state.phi;

	// Calculate controller outputs
	if((mb_state.theta>-CONTROLLER_SWITCH_ANGLE) || (mb_state.theta<CONTROLLER_SWITCH_ANGLE))
	{
	mb_state.u = mb_gains.K1*mb_state.theta + mb_gains.K2*mb_state.theta_dot - mb_gains.K3*mb_state.phi - mb_gains.K4*mb_state.phi_dot + mb_gains.Nbar*0.001;
	}
	else
	{
	mb_state.u = mb_gains.K1*1.2*mb_state.theta + mb_gains.K2*mb_state.theta_dot - mb_gains.K3*mb_state.phi - mb_gains.K4*mb_state.phi_dot + mb_gains.Nbar*0.001;

	}

	if(mb_state.u < -1)
	{
		mb_state.u = -0.999;
	}
	else if(mb_state.u > 1)
	{
		mb_state.u =0.999;
	}

	// if(mb_state.u < -0.1)
	// {
	// 	mb_motor_set(RIGHT_MOTOR,mb_state.u);
	// 	mb_motor_set(LEFT_MOTOR,min((mb_state.u *(float)(mb_gains.temp1)),-0.999));

	// }
	// else if (mb_state.u > 0.1)
	// {
	// 	mb_motor_set(RIGHT_MOTOR,mb_state.u);
	// 	mb_motor_set(LEFT_MOTOR,max((mb_state.u *(float)(mb_gains.temp1)),0.999));
	// }
	// else
	// {
		// mb_motor_set(RIGHT_MOTOR,mb_state.u);
		// mb_motor_set(LEFT_MOTOR,mb_state.u);

	// }

	
		

//	}
    
    if(!mb_setpoints.manual_ctl){
    	//send motor commands
   	}

    if(mb_setpoints.manual_ctl){
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
    pthread_mutex_unlock(&state_mutex);
	pthread_mutex_unlock(&gains_mutex);

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

			printf("%f",turn_stick);
			printf("/n");
			printf("%f",input_mode);
			printf("\n");
			printf("%f",drive_stick);
		
		}

	 	rc_nanosleep(1E9 / RC_CTL_HZ);
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
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |            MOCAP            |");
			printf("\n");
			printf("    θ    |");
			printf("    φ    |");
			printf("theta_dot |");
			printf("phi_dot   |");
			printf("u         |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			// printf("\r");
			// //Add Print stattements here, do not follow with /n
			// pthread_mutex_lock(&state_mutex);
			// printf("%7.3f  |", mb_state.theta);
			// printf("%7.3f  |", mb_state.phi);
			// printf("%7.3f  |", mb_state.theta_dot);
			// printf("%7.3f  |", mb_state.phi_dot);
			// printf("%7.3f  |", mb_state.u);
			// printf("%7d  |", mb_state.left_encoder);
			// printf("%7d  |", mb_state.right_encoder);
			// // printf("%7.3f  |", mb_state.opti_x);
			// // printf("%7.3f  |", mb_state.opti_y);
			// // printf("%7.3f  |", mb_state.opti_yaw);
			// printf("%7.3f  |", mb_odometry.x);
			// printf("%7.3f  |", mb_odometry.y);
			// printf("%7.3f  |", mb_odometry.psi);
			// pthread_mutex_unlock(&state_mutex);
			// fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 

void* set_gains(void* ptr) {
	while(rc_get_state()!=EXITING) {
		pthread_mutex_lock(&gains_mutex);
		int ch = getch();
		
		if (ch == ERR) 
			printf("no input\n");
		else {
			switch (ch) {
				case '1':
					mb_gains.K1 += mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '2':
					mb_gains.K1 -= mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '3':
					mb_gains.K2 += mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '4':
					mb_gains.K2 -= mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '5':
					mb_gains.K3 += mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '6':
					mb_gains.K3 -= mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '7':
					mb_gains.K4 += mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '8':
					mb_gains.K4 -= mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '9':
					mb_gains.Nbar += mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case '0':
					mb_gains.Nbar -= mb_gains.incre;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case 'q':
					mb_gains.incre += 0.01;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				case 'w':
					mb_gains.incre -= 0.01;
					printf("\n%f %f %f %f %f %f\n", mb_gains.K1, mb_gains.K2, mb_gains.K3, mb_gains.K4, mb_gains.Nbar, mb_gains.incre);
					break;
				default:
					printf("%c\n",ch);
					break;
			}
		}

		pthread_mutex_unlock(&gains_mutex);
		rc_nanosleep(1E9/5);
	}
	return NULL;
}