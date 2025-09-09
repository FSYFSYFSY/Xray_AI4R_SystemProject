/*

Messages are ASCII Encoded from the pico board as follows:

IMU G u_sec_since_last_poll yaw pitch roll
IMU A u_sec_since_last_poll accelx accely accelz 
IMU M u_sec_since_last_poll magx magy magz mag_accurcy
TOF tof_num u_sec_since_last_poll distance_mm_1 ... distance_mm_16

The approximate polling rates are currently:
- ~100Hz for the IMU
- ~7Hz Per TOF Sensor When running 8 of them
- ~14Hz Per TOF Sensor when running 8 of them

*/

#include <stdlib.h>
#include <string.h>
#include "pico/multicore.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "bno08x.h"
#include "utils.h"
extern "C" {
    #include "vl53l5cx_library/vl53l5cx_api.h"
}

BNO08x IMU;

// vl53l5cx i2c setup
#define I2C_SDA 2
#define I2C_SCL 3
i2c_inst_t vl53l5cx_i2c = {i2c1_hw, false};

// Run the TOF code on the second core of the RP2040
void core1_entry() {
    multicore_lockout_victim_init();

    /* Setup VL53L5CX */ 
    i2c_init(&vl53l5cx_i2c, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

	uint8_t status, isAlive, isReady;
	VL53L5CX_Configuration  Dev;			/* Sensor configuration */
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

	
	Dev.platform.address = (uint8_t)((VL53L5CX_DEFAULT_I2C_ADDRESS >> 1) & 0xFF);
    Dev.platform.i2c     = &vl53l5cx_i2c; // set i2c bus

    uint8_t one_hot_encoding[] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
    int enabled[8] = {0,0,0,0,0,0,0,0}, en_cnt = 0;

    // Try to enable a tof on each mux output, and add whether that position is enabled
    // to the enabled array can select which tof is selected by writing one byte to 
    // the mux with one hot encoding
    i2c_write_blocking(&vl53l5cx_i2c, 0x70, 0x00,1,false);

    for (int i = 0; i < 8; i++) {
        i2c_write_blocking(&vl53l5cx_i2c, 0x70, &one_hot_encoding[i],1,false);
        sleep_ms(5);
        status = vl53l5cx_is_alive(&Dev, &isAlive);
        if(!isAlive || status) {
            multicore_lockout_start_blocking();
            printf("skip %d\n",i);
            multicore_lockout_end_blocking();
            continue; //no tof found at this mux position
        }

        /* (Mandatory) Init VL53L5CX sensor */
        status = vl53l5cx_init(&Dev);
        status = vl53l5cx_set_resolution(&Dev, 16U);
        status = vl53l5cx_set_ranging_frequency_hz(&Dev, 30);
        if(status) {
            //printf("vl53l5cx_set_ranging_frequency_hz failed, status %u\n", status);
            continue;
        }
        status = vl53l5cx_start_ranging(&Dev);
        if(status) {
            //printf("vl53l5cx start ranging failed status %u\n", status);
            continue;
        }
        enabled[en_cnt] = i;
        en_cnt++;
        //printf("TOF %d Done\n",i);
    }

    // setup completed now

    absolute_time_t curr_time = get_absolute_time();
    absolute_time_t tof_timestamp[8] = {curr_time,curr_time,curr_time,curr_time,curr_time,curr_time,curr_time,curr_time};
    int i = 0, en = en_cnt;
    while(true) {
        // POLL TOF's
        en = (en >= en_cnt) ? 0 : en+1;
        i = enabled[en];
        // for efficency we only iterate through the enabled positions where (i) is 
        // the position on the mux that is to be enabled via sending a byte to the mux 
        i2c_write_blocking(&vl53l5cx_i2c, 0x70, &one_hot_encoding[i],1,false);
        vl53l5cx_is_alive(&Dev, &isAlive);
        status = vl53l5cx_check_data_ready(&Dev, &isReady);
        if(isReady) {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            multicore_lockout_start_blocking();
            printf("TOF %d %lld",i, absolute_time_diff_us(tof_timestamp[i],curr_time));
            for(int j = 0; j < 16; j++) {
                printf(" %d", Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*j]);
            }
            printf("\n");
            multicore_lockout_end_blocking();

            tof_timestamp[i] = curr_time;
            curr_time = get_absolute_time();
        }
        
    }
}


int main(void) {
    // pico settings
    stdio_init_all();
    sleep_ms(3000);
    // enable the second core and allow for multicore lockout to prevent double printing to stdout
    multicore_lockout_victim_init();
    multicore_launch_core1(core1_entry);
    
    /* Setup IMU (BNO08X) */
    i2c_inst_t* i2c0_imu;
    initI2C(i2c0_imu, false);
    
    while (IMU.begin(CONFIG::BNO08X_ADDR, i2c0_imu) == false) {
        multicore_lockout_start_blocking();
        printf("BNO08x not detected at default I2C address. Check wiring. Freezing\n");
        multicore_lockout_end_blocking();
        sleep_ms(1000);
    }
    IMU.enableRotationVector();
    IMU.enableAccelerometer();
    IMU.enableMagnetometer();
    IMU.tareNow();
    IMU.saveTare();

    absolute_time_t curr_time = get_absolute_time();
    absolute_time_t imu_gyro_timestamp = curr_time;
    absolute_time_t imu_accel_timestamp = curr_time;
    absolute_time_t imu_mag_timestamp = curr_time;
    double yaw, pitch, roll, ax, ay, az, mx, my, mz;
    uint8_t m_accur;
	while(true) {   
        // POLL IMU
		curr_time = get_absolute_time();
        if (IMU.wasReset()) {
            multicore_lockout_start_blocking();
            printf("sensor was reset\n");
            multicore_lockout_end_blocking();
            IMU.enableRotationVector();
            IMU.enableAccelerometer();
            IMU.enableMagnetometer();
        }
		
        if (IMU.getSensorEvent() == true) {
            if (IMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
                yaw = IMU.getYaw();
                pitch = IMU.getPitch();
                roll = IMU.getRoll();
                multicore_lockout_start_blocking();
                printf("IMU G %lld ",absolute_time_diff_us(imu_gyro_timestamp,curr_time));
				printf("%.2lf %.2lf %.2lf\n", yaw, pitch, roll);
                multicore_lockout_end_blocking();

                imu_gyro_timestamp = curr_time;
				curr_time = get_absolute_time();
            } else if (IMU.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
				ax = IMU.getAccelX();
                ay = IMU.getAccelY();
                az = IMU.getAccelZ();
                multicore_lockout_start_blocking();
                printf("IMU A %lld ",absolute_time_diff_us(imu_accel_timestamp,curr_time));
				printf("%.2lf %.2lf %.2lf\n", ax, ay, az);
                multicore_lockout_end_blocking();

                imu_accel_timestamp = curr_time;
				curr_time = get_absolute_time();
            } else if (IMU.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD){
				mx = IMU.getMagX();
                my = IMU.getMagY();
                mz = IMU.getMagZ();
                m_accur = IMU.getMagAccuracy();
                multicore_lockout_start_blocking();
                printf("IMU M %lld ",absolute_time_diff_us(imu_mag_timestamp,curr_time));
				printf("%.2lf %.2lf %.2lf %d\n", mx, my, mz, m_accur);
                multicore_lockout_end_blocking();

                imu_mag_timestamp = curr_time;
				curr_time = get_absolute_time();
            }
		}
	}

	return 0;
}
