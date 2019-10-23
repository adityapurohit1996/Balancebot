/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
#define _STDC_FORMAT_MACROS


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>
#include <inttypes.h>



uint64_t t_pre, t_cur;
rc_mpu_data_t mpu_data;

//call back function
void imu_data() {
    t_cur = rc_nanos_since_epoch();
    float t_diff = (float)((t_cur-t_pre)/1E9);   
    FILE* f1;
    f1 = fopen("/home/debian/gyro_data.csv", "a");
    rc_mpu_read_gyro(&mpu_data);
    // fprintf(f1, "%d %f %f %f %f %f %f %f %f %f\n", t_diff, mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2], mpu_data.accel[0], mpu_data.accel[1], mpu_data.accel[2],mpu_data.dmp_TaitBryan[0], mpu_data.dmp_TaitBryan[1],mpu_data.dmp_TaitBryan[2] );
    // printf("%d %f %f %f %f %f %f %f %f %f\n", t_diff, mpu_data.gyror[0], mpu_data.gyro[1], mpu_data.gyro[2], mpu_data.accel[0], mpu_data.accel[1], mpu_data.accel[2],mpu_data.dmp_TaitBryan[0], mpu_data.dmp_TaitBryan[1],mpu_data.dmp_TaitBryan[2] );
    fprintf(f1, "%f, %f, %f, %f, %f, %f, %f\n", t_diff, mpu_data.dmp_TaitBryan[0], mpu_data.dmp_TaitBryan[1], mpu_data.dmp_TaitBryan[2], mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2]);
    printf("%f, %f, %f, %f, %f, %f, %f\n", t_diff, mpu_data.dmp_TaitBryan[0], mpu_data.dmp_TaitBryan[1], mpu_data.dmp_TaitBryan[2], mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2]);
    fclose(f1);
}

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){

	
    t_pre = rc_nanos_since_epoch();
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

    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    //rc_mpu_data_t mpu_data;
	// mpu_config.dmp_sample_rate = 100;
	// mpu_config.orient = ORIENTATION_Z_UP;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
    rc_mpu_set_dmp_callback(&imu_data);

    rc_set_state(RUNNING);
    
    // uint64_t t_pre, t_cur;
    // t_pre = rc_nanos_since_epoch();
    while(rc_get_state()!=EXITING){
        // rc_mpu_read_gyro(&mpu_data);
        // rc_mpu_read_accel(&mpu_data);

        // t_cur = rc_nanos_since_epoch();
        // uint64_t t_diff = t_cur-t_pre;
        // printf("%"PRIu64"\n", t_diff);
        // time in milliseconds
        // int t_diff = (int)(t_cur-t_pre);   
        // t_pre = t_cur;
        // printf("gyro data:%f %f %f \n", mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2]);
        // printf("t_diff:%d\n", t_diff);
        // fprintf(f1, "%d %f %f %f %f %f %f %f %f %f\n", t_diff, mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2], mpu_data.accel[0], mpu_data.accel[1], mpu_data.accel[2], mpu_data.dmp_TaitBryan[0], mpu_data.dmp_TaitBryan[1],mpu_data.dmp_TaitBryan[2]) ;
    	//rc_nanosleep(1E8);
    }

    
	// exit cleanly
	rc_encoder_eqep_cleanup();
    rc_mpu_power_off();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}