/*

Messages are ASCII Encoded from the pico board as follows:

IMU G u_sec_since_last_poll yaw pitch roll
IMU A u_sec_since_last_poll accelx accely accelz 
IMU M u_sec_since_last_poll magx magy magz mag_accurcy
TOF tof_num u_sec_since_last_poll distance_mm_1 ... distance_mm_16

The approximate polling rates are currently:
- ~100Hz for the IMU

*/

#include <stdlib.h>
#include <string.h>
//#include "pico/multicore.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "bno08x.h"
#include "utils.h"

BNO08x IMU;



int main(void) {
    // pico settings
    stdio_init_all();
    sleep_ms(3000);
    // enable the second core and allow for multicore lockout to prevent double printing to stdout
    //multicore_lockout_victim_init();
    //multicore_launch_core1(core1_entry);
    
    // ==================
    // Setup IMU (BNO08X)
    i2c_inst_t* i2c0_imu;
    initI2C(i2c0_imu, false);
    
    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c0_imu) == false) {
        //multicore_lockout_start_blocking();
        printf("[MAIN] BNO08x sensor not detected at default I2C address. Trying again in 1 second.\n");
        //multicore_lockout_end_blocking();
        sleep_ms(1000);
    }
    IMU.enableRotationVector();
    IMU.enableAccelerometer();
    IMU.enableMagnetometer();
    IMU.tareNow();
    IMU.saveTare();

    // =====================
    // PREPARE IMU VARIABLES
    absolute_time_t curr_time = get_absolute_time();
    absolute_time_t imu_gyro_timestamp = curr_time;
    absolute_time_t imu_accel_timestamp = curr_time;
    absolute_time_t imu_mag_timestamp = curr_time;
    double yaw, pitch, roll, ax, ay, az, mx, my, mz;
    uint8_t m_accur;

    // ========
    // POLL IMU
	while(true) {   
        curr_time = get_absolute_time();
        if (IMU.wasReset()) {
            //multicore_lockout_start_blocking();
            printf("[MAIN] BNO08x sensor was reset\n");
            //multicore_lockout_end_blocking();
            IMU.enableRotationVector();
            IMU.enableAccelerometer();
            IMU.enableMagnetometer();
        }
		
        if (IMU.getSensorEvent() == true) {
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                yaw = IMU.getYaw();
                pitch = IMU.getPitch();
                roll = IMU.getRoll();
                //multicore_lockout_start_blocking();
                printf("IMU G %lld ",absolute_time_diff_us(imu_gyro_timestamp,curr_time));
				printf("%.2lf %.2lf %.2lf\n", yaw, pitch, roll);
                //multicore_lockout_end_blocking();

                imu_gyro_timestamp = curr_time;
				curr_time = get_absolute_time();
            } else if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
				ax = IMU.getAccelX();
                ay = IMU.getAccelY();
                az = IMU.getAccelZ();
                //multicore_lockout_start_blocking();
                printf("IMU A %lld ",absolute_time_diff_us(imu_accel_timestamp,curr_time));
				printf("%.2lf %.2lf %.2lf\n", ax, ay, az);
                //multicore_lockout_end_blocking();

                imu_accel_timestamp = curr_time;
				curr_time = get_absolute_time();
            } else if (IMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD){
				mx = IMU.getMagX();
                my = IMU.getMagY();
                mz = IMU.getMagZ();
                m_accur = IMU.getMagAccuracy();
                //multicore_lockout_start_blocking();
                printf("IMU M %lld ",absolute_time_diff_us(imu_mag_timestamp,curr_time));
				printf("%.2lf %.2lf %.2lf %d\n", mx, my, mz, m_accur);
                //multicore_lockout_end_blocking();

                imu_mag_timestamp = curr_time;
				curr_time = get_absolute_time();
            }
		}
	}

	return 0;
}
