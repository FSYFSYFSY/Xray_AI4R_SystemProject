/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */



#include "vl53l5cx/platform/vl53l5cx_platform.h"





uint8_t VL53L5CX_RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t register_address,
		uint8_t *read_data_array)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */

	// Call the "Read Multi" for 1 byte of data
	return VL53L5CX_RdMulti(p_platform, register_address, read_data_array, 1);
}

uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t register_address,
		uint8_t write_data)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */

	// Call the "Write Multi" for 1 byte of data
	return VL53L5CX_WrMulti(p_platform, register_address, (uint8_t *) &write_data, 1);
}

uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t register_address,
		uint8_t *write_data_array,
		uint32_t num_write_bytes)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */

	//printf("DEBUGGING: VL53L1_WriteMulti function called with register_address = %d\n", register_address);

	// Hardcode the address of the VL53L1X for now
	//uint8_t i2c_address = 0x52;

	// Get the I2C address of this sensor
	uint8_t i2c_address = p_platform->address;

	// Get the file descriptor of the I2C bus to which
	// this sensor is connected
	uint16_t i2c_bus_fd = p_platform->i2c_fd;

	// WRITE THE DATA IN CHUNKS UNTIL "num_write_bytes"
	// > The majority of communications are less than the
	//   chunk size of "VL53L5CX_MAX_I2C_TRANSFER_SIZE"
	// > The VL53L5CX sensor however requires writing the
	//   firmware to the sensor on every startup, which
	//   requires writing approximately 88 kilo bytes.

	// Initialise variable for counting the chucks
	uint32_t data_size = 0;
	uint16_t position = 0;

	// Inform the user if writing in chunks
	if (num_write_bytes > (VL53L5CX_MAX_I2C_TRANSFER_SIZE-2))
	{
		// Compute the rounded up number of chunks without an float cast
		int num_chunks = (num_write_bytes+(VL53L5CX_MAX_I2C_TRANSFER_SIZE-2)-1) / (VL53L5CX_MAX_I2C_TRANSFER_SIZE-2);
		// Display the message
		printf("[VL53L5CX PLATFORM] Now starting to write %d bytes in chunks of size %d bytes, i.e., %d chunks.\n", num_write_bytes, (VL53L5CX_MAX_I2C_TRANSFER_SIZE-2), num_chunks);
	}

	// Continue until the "position" of writing
	// equals the number of bytes to write
	while (position < num_write_bytes)
	{
		// Convert the "register_address" into its two
		// separate bytes
		// > The first data byte written contains the
		//   most significant eight bits of the register
		//   address
		// > The second data byte written contains the
		//   least significant eight bits of the register
		//   address
		uint8_t reg_addr_byte_msb = (register_address+position) >> 8;
		uint8_t reg_addr_byte_lsb = (register_address+position) & 0xFF;

		// Compute the number of bytes to transfer this time
		if ((num_write_bytes - position) > (VL53L5CX_MAX_I2C_TRANSFER_SIZE-2))
			data_size = (VL53L5CX_MAX_I2C_TRANSFER_SIZE-2);
		else
			data_size = (num_write_bytes - position);

		// Put the "register address" into the uint8
		// write array
		vl53l5cx_write_buffer[0] = reg_addr_byte_msb;
		vl53l5cx_write_buffer[1] = reg_addr_byte_lsb;

		// Copy the values pointed to by "write_data_array"
		// into the uint8 write array
		memcpy(&vl53l5cx_write_buffer[2], &write_data_array[position], data_size);
		// Specify the number of bytes in the array
		uint16_t this_num_write_bytes_total = data_size + 2;

		// Create an array of "i2c_msg structs" with:
		// > One message for the data to write
		struct i2c_msg message = { i2c_address, 0, this_num_write_bytes_total, vl53l5cx_write_buffer };

		// Create the struct for using the ioctl interface
		struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

		// Call the ioctl interface
		int result = ioctl(i2c_bus_fd, I2C_RDWR, &ioctl_data);

		// Check the result of the ioctl call
		if (result != 1)
		{
			// Inform the user
			//perror("FAILED result from call to ioctl.\n");
			// Return flag that ioctl was unsuccessful
			status = -1;
			return status;
		}

		// Increment the write position
		position +=  data_size;
	}

	// Return flag that ioctl was successful
	//printf("SUCCESS result from call to ioctl.\n");
	status = 0;
	return status;
}

uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t register_address,
		uint8_t *read_data_array,
		uint32_t num_read_bytes)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */

	//printf("DEBUGGING: VL53L1_ReadMulti function called with register_address = %d\n", register_address);

	// Hardcode the address of the VL53L1X for now
	//uint8_t i2c_address = 0x52;

	// Get the I2C address of this sensor
	uint8_t i2c_address = p_platform->address;

	// Get the file descriptor of the I2C bus to which
	// this sensor is connected
	uint16_t i2c_bus_fd = p_platform->i2c_fd;

	// READ THE DATA IN CHUNKS UNTIL "num_read_bytes"
	// > The majority of communications are less than the
	//   chunk size of "VL53L5CX_MAX_I2C_TRANSFER_SIZE"
	// > The VL53L5CX sensor API offers some rarely used
	//   functions that require this.
	// > The main use case for this is reading the ranging
	//   results.

	// Initialise variable for counting the chucks
	uint32_t data_size = 0;
	uint16_t position = 0;

	// Inform the user if reading in chunks
	if (num_read_bytes > VL53L5CX_MAX_I2C_TRANSFER_SIZE)
	{
		// Compute the rounded up number of chunks without an float cast
		int num_chunks = (num_read_bytes+VL53L5CX_MAX_I2C_TRANSFER_SIZE-1) / (VL53L5CX_MAX_I2C_TRANSFER_SIZE);
		// Display the message
		printf("[VL53L5CX PLATFORM] Now starting to read %d bytes in chunks of size %d bytes, i.e., %d chunks.\n", num_read_bytes, VL53L5CX_MAX_I2C_TRANSFER_SIZE, num_chunks);
	}

	// Continue until the "position" of reading
	// equals the number of bytes to read
	while (position < num_read_bytes)
	{
		// Convert the "register_address" into its two
		// separate bytes
		// > The first data byte written contains the
		//   most significant eight bits of the register
		//   address
		// > The second data byte written contains the
		//   least significant eight bits of the register
		//   address
		uint8_t reg_addr_byte_msb = (register_address+position) >> 8;
		uint8_t reg_addr_byte_lsb = (register_address+position) & 0xFF;

		// Put the "register address" into the uint8
		// write array
		vl53l5cx_write_buffer[0] = reg_addr_byte_msb;
		vl53l5cx_write_buffer[1] = reg_addr_byte_lsb;

		// Specify the number of bytes in the array
		uint16_t num_write_bytes_total = 2;

		// Compute the number of bytes to transfer this time
		if ((num_read_bytes - position) > (VL53L5CX_MAX_I2C_TRANSFER_SIZE))
			data_size = (VL53L5CX_MAX_I2C_TRANSFER_SIZE);
		else
			data_size = (num_read_bytes - position);

		// Cast the number of read bytes to 16-bit
		// becuase that is what "ioctl" expects
		uint16_t num_read_bytes_uint16 = (uint16_t)data_size;

		// Create an array of "i2c_msg structs" with:
		// > First message for the data to write
		// > Second message for reading data
		struct i2c_msg messages[] = {
			{ i2c_address, 0       , num_write_bytes_total, vl53l5cx_write_buffer },
			{ i2c_address, I2C_M_RD, num_read_bytes_uint16, (read_data_array+position) },
		};

		// Create the struct for using the ioctl interface
		struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

		// Call the ioctl interface
		int result = ioctl(i2c_bus_fd, I2C_RDWR, &ioctl_data);

		// Check the result of the ioctl call
		if (result != 2)
		{
			// Inform the user
			//perror("FAILED result from call to ioctl.\n");
			// Return flag that ioctl was unsuccessful
			status = -1;
			return status;
		}

		// Increment the read position
		position +=  data_size;
	}

	// Return flag that ioctl was successful
	//printf("SUCCESS result from call to ioctl.\n");
	status = 0;
	return status;
}

uint8_t VL53L5CX_Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;

	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO to LOW */
	VL53L5CX_WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of to HIGH */
	VL53L5CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L5CX_SwapBuffer(
		uint8_t *buffer,
		uint16_t size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
	usleep(TimeMs * 1000);
	status = 0;

	return status;
}
