// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
// This file is part of ASClinic-System.
//    
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \.
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// I2C driver for the VL53L5CX time-of-flight distance sensor
//
// ----------------------------------------------------------------------------





#ifndef VL53L5CX_CPP_INTERFACE_H
#define VL53L5CX_CPP_INTERFACE_H





// #include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "vl53l5cx/platform/vl53l5cx_platform.h"
#include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_api.h"
#ifdef __cplusplus
}
#endif
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_detection_thresholds.h
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_motion_indicator.h
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_buffers.h

#include "i2c_driver/i2c_driver.h"





// I2C ADDRESS
#define VL53L5CX_I2C_ADDRESS_DEFAULT    0x29 /**< Default VL53L5CX I2C Slave Address */
#define TCA9548A_I2C_ADDRESS_DEFAULT    0x70 /**< Default TCA9548A I2C Slave Address */





// ----------------------------------------------------------
//   CCCC  L        A     SSSS   SSSS     DDDD   EEEEE  FFFFF
//  C      L       A A   S      S         D   D  E      F
//  C      L      A   A   SSS    SSS      D   D  EEE    FFF
//  C      L      AAAAA      S      S     D   D  E      F
//   CCCC  LLLLL  A   A  SSSS   SSSS      DDDD   EEEEE  F
// ----------------------------------------------------------

class VL53L5CX
{
	// ----------------------------------
	//  EEEEE  N   N  U   U  M   M   SSSS
	//  E      NN  N  U   U  MM MM  S
	//  EEE    N N N  U   U  M M M   SSS
	//  E      N  NN  U   U  M   M      S
	//  EEEEE  N   N   UUU   M   M  SSSS
	// ----------------------------------

	enum class State : int
	{
		uninitialised = 0,
		initialising = 1,
		idle = 2,
		ranging = 3,
	};





	// ------------------------------------------------------------
	//  V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
	//  V   V   A A   R   R   I    A A   B   B  L      E      S
	//  V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
	//   V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
	//    V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
	// ------------------------------------------------------------

private:
	uint8_t m_i2c_address;
	I2C_Driver * m_i2c_driver;

	bool m_is_connected_to_TCA9548A_mux;
	uint8_t m_mux_channel;
	uint8_t m_mux_i2c_address;

	uint8_t m_sensor_id;

	VL53L5CX::State m_state;

	// The "platform" struct used by the driver
	// provided by ST.
	// > Only the ".platform" property of this
	//   struct should be edited.
	// > The ".platform" field is used for passing
	//   around the sensor's I2C address and
	//   the file descriptor of the I2C bus to
	//   which the sensor is connected
	VL53L5CX_Configuration m_vl53l5cx_configuration;





	// ----------------------------------------------------------------------------
	//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
	//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
	//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
	//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
	//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
	// ----------------------------------------------------------------------------
public:
	/**
	 * @brief Simple constructor.
	 *
	 * @param (I2C_Driver*) i2c_driver : The I2C driver object to which the sensor is connected.
	 */
	VL53L5CX(I2C_Driver * i2c_driver);

	/**
	 * @brief Constructor with I2C device address specified.
	 *
	 * @param (I2C_Driver*) i2c_driver : The I2C driver object to which the sensor is connected.
	 * @param (uint8_t) address : I2C address of the sensor.
	 */
	VL53L5CX(I2C_Driver * i2c_driver, uint8_t address);
	
	/**
	 * @brief Constructor with I2C device address and multiplexer details specified.
	 *
	 * @param (I2C_Driver*) i2c_driver : The I2C driver object to which the sensor is connected.
	 * @param (uint8_t) address : I2C address of the sensor.
	 * @param (uint8_t) mux_channel : The multiplexer channel to which the sensor is connected.
	 * @param (uint8_t) mux_i2c_address : I2C address of the multiplexer,
	 */
	VL53L5CX(I2C_Driver * i2c_driver, uint8_t address, uint8_t mux_channel, uint8_t mux_i2c_address);





	// ------------------------------------------------------
	//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
	//  G      E        T        &        S      E        T
	//  G  GG  EEE      T        && &      SSS   EEE      T
	//  G   G  E        T       &  &          S  E        T
	//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
	// ------------------------------------------------------
public:
	uint8_t get_i2c_address();
	bool set_i2c_address(uint8_t new_address);

	uint8_t get_mux_channel();
	bool set_mux_channel(uint8_t new_channel);

	uint8_t get_mux_i2c_address();
	bool set_mux_i2c_address(uint8_t new_address);

	// ------------------------------------------------------------
	//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
	//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
	//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
	//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
	//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
	// ------------------------------------------------------------

public:
	bool switch_mux_channel_to_this_sensor();

	bool read_register(uint16_t dev, uint16_t register_address, uint16_t * value);

	bool write_register(uint16_t dev, uint16_t register_address, uint16_t value);

// Calls to the VL53L5CX API provided by ST
public:

	bool sensor_is_alive(uint8_t * is_alive);
	
	bool sensor_init();

	bool get_power_mode(uint8_t * power_mode);

	bool set_power_mode(uint8_t power_mode);

	bool start_ranging();

	bool stop_ranging();

	bool check_data_ready(uint8_t * is_ready);

	bool get_ranging_data(VL53L5CX_ResultsData * results_data);

	bool get_resolution(uint8_t * resolution);

	bool set_resolution(uint8_t resolution);

	bool get_ranging_frequency_hz(uint8_t * frequency_hz);

	bool set_ranging_frequency_hz(uint8_t frequency_hz);

	bool get_integration_time_ms(uint32_t * integration_time_ms);

	bool set_integration_time_ms(uint32_t integration_time_ms);

	bool get_sharpener_percent(uint8_t * sharpener_percent);

	bool set_sharpener_percent(uint8_t sharpener_percent);

	bool get_target_order(uint8_t * target_order);

	bool set_target_order(uint8_t target_order);

	bool get_ranging_mode(uint8_t * ranging_mode);

	bool set_ranging_mode(uint8_t ranging_mode);

	bool enable_internal_cp();

	bool disable_internal_cp();





// Convenience Functions
public:
	bool initialise(uint8_t resolution, uint8_t ranging_frequency_hz, uint8_t integration_time_ms, uint8_t sharpener_percent, uint8_t target_order, uint8_t ranging_mode);

	bool initialise_and_start_ranging(uint8_t resolution, uint8_t ranging_frequency_hz, uint8_t integration_time_ms, uint8_t sharpener_percent, uint8_t target_order, uint8_t ranging_mode);

	bool get_distance_measurement(VL53L5CX_ResultsData * results_data);

	bool check_data_ready_multiple_then_get_distance_measurement(VL53L5CX_ResultsData * results_data, uint16_t interval_between_checks_usec, uint16_t number_of_checks);


}; // END OF CLASS DEFINITION





#endif // VL53L5CX_CPP_INTERFACE_H


