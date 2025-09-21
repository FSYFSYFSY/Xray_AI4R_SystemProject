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
    // Mark start of core1 execution for debug visibility
    multicore_lockout_start_blocking();
    printf("[CORE1] started TOF task\n");
    multicore_lockout_end_blocking();

    /* Setup VL53L5CX */ 
    multicore_lockout_start_blocking();
    printf("[CORE1] init I2C1 begin\n");
    multicore_lockout_end_blocking();

    // Attempt bus recovery on I2C1 in case lines are held
    i2cBusRecovery(I2C_SDA, I2C_SCL);

    i2c_init(&vl53l5cx_i2c, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    multicore_lockout_start_blocking();
    printf("[CORE1] init I2C1 done\n");
    multicore_lockout_end_blocking();

    // Quick I2C1 probe for mux (0x70) using a safe 1-byte write
    // {
    //     uint8_t zero = 0x00;
    //     int ret70 = i2c_write_blocking(&vl53l5cx_i2c, 0x70, &zero, 1, false);
    //     multicore_lockout_start_blocking();
    //     printf("I2C1 probe: 0x70 write ret=%d\n", ret70);
    //     multicore_lockout_end_blocking();
    //     // Repeat a few times to ensure visibility even if host attaches late
    //     for (int k = 1; k <= 6; ++k) {
    //         ret70 = i2c_write_blocking(&vl53l5cx_i2c, 0x70, &zero, 1, false);
    //         multicore_lockout_start_blocking();
    //         printf("I2C1 probe[%d]: 0x70 write ret=%d\n", k, ret70);
    //         multicore_lockout_end_blocking();
    //         sleep_ms(500);
    //     }
    // }

	uint8_t status, isAlive, isReady;
	VL53L5CX_Configuration  Dev;			/* Sensor configuration */
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */

	
	Dev.platform.address = (uint8_t)((VL53L5CX_DEFAULT_I2C_ADDRESS >> 1) & 0xFF);
    Dev.platform.i2c     = &vl53l5cx_i2c; // set i2c bus

    uint8_t one_hot_encoding[] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
    int enabled[8] = {0,0,0,0,0,0,0,0}, en_cnt = 0;

    // Try to detect presence of the I2C mux (0x70) via a write
    // > If absent, we assume a single VL53L5CX on the bus and skip mux ops
    // > If present, we check for a VL53L5CX at each of the 8 mux ports

    multicore_lockout_start_blocking();
    printf("[CORE1] Now probing for presence of a i2c mux\n");
    multicore_lockout_end_blocking();

    uint8_t probe_zero = 0x00;
    int mux_probe = i2c_write_blocking(&vl53l5cx_i2c, 0x70, &probe_zero, 1, false);
    bool has_mux = (mux_probe >= 0);

    if (has_mux) {
        multicore_lockout_start_blocking();
        printf("[CORE1] Mux found at addres 0x70\n");
        multicore_lockout_end_blocking();
    } else {
        multicore_lockout_start_blocking();
        printf("[CORE1] Mux not found at addres 0x70\n");
        multicore_lockout_end_blocking();
    }

    for (int i = 0; i < 8; i++) {
        // If there's no mux, only attempt index i==0 and skip others
        if (!has_mux && i != 0) {
            continue;
        }

        if (has_mux) {
            i2c_write_blocking(&vl53l5cx_i2c, 0x70, &one_hot_encoding[i], 1, false);
        }
        sleep_ms(5);
        status = vl53l5cx_is_alive(&Dev, &isAlive);
        if(!isAlive || status) {
            multicore_lockout_start_blocking();
            printf("[CORE1] skipping vl53l5cx for i= %d\n",i);
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
        if (has_mux) {
            en = (en >= en_cnt) ? 0 : en + 1;
            i = enabled[en];
            // for efficiency we only iterate through enabled positions where (i) is
            // the position on the mux that is to be enabled via sending a byte
            i2c_write_blocking(&vl53l5cx_i2c, 0x70, &one_hot_encoding[i], 1, false);
        } else {
            // No mux: single sensor assumed; use logical index 0 and skip mux writes
            i = 0;
        }
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
        } else {
            // Avoid tight spinning when data not ready; reduce contention
            sleep_ms(1);
            continue;
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
        printf("[MAIN] BNO08x sensor not detected at default I2C address. Trying again in 1 second.\n");
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
            printf("[MAIN] BNO08x sensor was reset\n");
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
