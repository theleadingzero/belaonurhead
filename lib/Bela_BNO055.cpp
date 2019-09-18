/*
Bela_BNO055.cpp

by Becky Stewart 2017



This code is based on:
MPR121 library for Bela https://github.com/BelaPlatform/Bela/tree/master/examples/06-Sensors/capacitive-touch
MrHeadTracker https://git.iem.at/DIY/MrHeadTracker
Adafruit BNO055 Sensor library https://github.com/adafruit/Adafruit_BNO055


This software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt

---------------------------------------------------------------
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
  Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
  Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/


#include "Bela_BNO055.h"
#include <iostream>

/**************************************************************************
	I2C_BNO055
    Default constructor
**************************************************************************/
I2C_BNO055::I2C_BNO055() 
{

}

/**************************************************************************
	begin
    Handles initializing the sensor
**************************************************************************/
boolean I2C_BNO055::begin(uint8_t bus, uint8_t i2caddr) 
{
  _i2c_address = i2caddr;
	
	// begin I2C communication
  	if(initI2C_RW(bus, i2caddr, 0) > 0)
  		return false;
	
	// check the chip ID  
  	uint8_t id = readRegister(BNO055_CHIP_ID_ADDR);
  	rt_printf("id: %d\n", id);
  	if(id != BNO055_ID)
  	{
		usleep(1000); // hold on for boot
    	id = readRegister(BNO055_CHIP_ID_ADDR);
    	if(id != BNO055_ID) {
      		return false;  // still not? ok bail
    	}
  	}

  	// switch to config mode (just in case since this is the default) */
  	setMode(OPERATION_MODE_CONFIG);
  	rt_printf("switching to operation mode\n");

  	// reset 
  	writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x20);
  	usleep(300000); // hold on for boot
  	while (readRegister(BNO055_CHIP_ID_ADDR) != BNO055_ID)// change back to while
  	{
    	usleep(300000);
  	}
  	rt_printf("in operation mode\n");

  	// set to normal power mode
  	writeRegister(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  	usleep(10);

  	writeRegister(BNO055_PAGE_ID_ADDR, 0);

  // set the output units 
  /*uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  writeRegister(BNO055_UNIT_SEL_ADDR, unitsel);

  // Configure axis mapping (see section 3.4)
  writeRegister(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  usleep(10);
  writeRegister(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  usleep(10);*/
  
  writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x0);
  usleep(10);
  // Set the requested operating mode (see section 3.3) 
  setMode(OPERATION_MODE_IMUPLUS);
  usleep(20);

  return true;
}

/**************************************************************************
	setMode
    Puts the chip in the specified operating mode
**************************************************************************/
void I2C_BNO055::setMode(i2c_bno055_opmode_t mode)
{
  _mode = mode;
  writeRegister(BNO055_OPR_MODE_ADDR, _mode);
  usleep(30);
}

/**************************************************************************/
/*
  setExtCrystalUse
    Use the external 32.768KHz crystal
*/
/**************************************************************************/
void I2C_BNO055::setExtCrystalUse(boolean usextal)
{
  i2c_bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  usleep(25);
  writeRegister(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  usleep(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  usleep(20);
}

/**************************************************************************
	getSystemStatus
    Retrieves system status information from the sensor
**************************************************************************/
void I2C_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  writeRegister(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = readRegister(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = readRegister(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error     = readRegister(BNO055_SYS_ERR_ADDR);

  usleep(200);
}

/**************************************************************************
	getCalibration
    Returns the calibration statuses for the sensor
**************************************************************************/
void I2C_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) 
{
  uint8_t calData = readRegister(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/**************************************************************************
	readRegister
    Reads from requested register
**************************************************************************/
uint8_t I2C_BNO055::readRegister(uint8_t reg) 
{
    i2c_char_t inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = _i2c_address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = _i2c_address;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    int t = ioctl(i2C_file, I2C_RDWR, &packets);
    if( t < 0) {
        rt_printf("Unable to send data from readRegister: %d\n", t);
        return 0;
    }

    return inbuf;
}


/**************************************************************************
	writeRegister
    Writes to register
**************************************************************************/
void I2C_BNO055::writeRegister(uint8_t reg, uint8_t value) 
{
	uint8_t buf[2] = { reg, value };

	if(write(i2C_file, buf, 2) != 2)
	{
		std::cout << "Failed to write register " << (int)reg << " on BNO055\n";
		return;
	}
}

/**************************************************************************
	getQuat
    Get sensor quaternion data
**************************************************************************/
imu::Quaternion I2C_BNO055::getQuat(void)
{
  int16_t x, y, z, w;
  x = y = z = w = 0;

  // read quat data
  uint8_t w_lsb = readRegister(BNO055_QUATERNION_DATA_W_LSB_ADDR);
  uint8_t w_msb = readRegister(BNO055_QUATERNION_DATA_W_MSB_ADDR);
  uint8_t x_lsb = readRegister(BNO055_QUATERNION_DATA_X_LSB_ADDR);
  uint8_t x_msb = readRegister(BNO055_QUATERNION_DATA_X_MSB_ADDR);
  uint8_t y_lsb = readRegister(BNO055_QUATERNION_DATA_Y_LSB_ADDR);
  uint8_t y_msb = readRegister(BNO055_QUATERNION_DATA_Y_MSB_ADDR);
  uint8_t z_lsb = readRegister(BNO055_QUATERNION_DATA_Z_LSB_ADDR);
  uint8_t z_msb = readRegister(BNO055_QUATERNION_DATA_Z_MSB_ADDR);
  
  
  w = (((uint16_t)w_msb) << 8) | ((uint16_t)w_lsb);
  x = (((uint16_t)x_msb) << 8) | ((uint16_t)x_lsb);
  y = (((uint16_t)y_msb) << 8) | ((uint16_t)y_lsb);
  z = (((uint16_t)z_msb) << 8) | ((uint16_t)z_lsb);

  /* Assign to Quaternion */
  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
  const double scale = (1.0 / (1<<14));
  imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  return quat;
}


/**************************************************************************
	getVector
    Get sensor vector reading - currently only works for gravity
**************************************************************************/
imu::Vector<3> I2C_BNO055::getVector(i2c_vector_type_t vector_type)
{
  imu::Vector<3> xyz;

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  
  uint8_t x_lsb = readRegister(BNO055_GRAVITY_DATA_X_LSB_ADDR);
  uint8_t x_msb = readRegister(BNO055_GRAVITY_DATA_X_MSB_ADDR);
  uint8_t y_lsb = readRegister(BNO055_GRAVITY_DATA_Y_LSB_ADDR);
  uint8_t y_msb = readRegister(BNO055_GRAVITY_DATA_Y_MSB_ADDR);
  uint8_t z_lsb = readRegister(BNO055_GRAVITY_DATA_Z_LSB_ADDR);
  uint8_t z_msb = readRegister(BNO055_GRAVITY_DATA_Z_MSB_ADDR);

  x = ((int16_t)x_lsb) | (((int16_t)x_msb) << 8);
  y = ((int16_t)y_lsb) | (((int16_t)y_msb) << 8);
  z = ((int16_t)z_lsb) | (((int16_t)z_msb) << 8);

  /* Convert the value to an appropriate range (section 3.6.4) */
  /* and assign the value to the Vector type */
  switch(vector_type)
  {
    case VECTOR_MAGNETOMETER:
      /* 1uT = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_GYROSCOPE:
      /* 1dps = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_EULER:
      /* 1 degree = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_ACCELEROMETER:
    case VECTOR_LINEARACCEL:
    case VECTOR_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x)/100.0;
      xyz[1] = ((double)y)/100.0;
      xyz[2] = ((double)z)/100.0;
      break;
  }

  return xyz;
}


