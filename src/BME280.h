#ifndef __BME280_H__
#define __BME280_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

//Register names:
#define BME280_DIG_T1_LSB_REG 0x88
#define BME280_DIG_T1_MSB_REG 0x89
#define BME280_DIG_T2_LSB_REG 0x8A
#define BME280_DIG_T2_MSB_REG 0x8B
#define BME280_DIG_T3_LSB_REG 0x8C
#define BME280_DIG_T3_MSB_REG 0x8D
#define BME280_DIG_P1_LSB_REG 0x8E
#define BME280_DIG_P1_MSB_REG 0x8F
#define BME280_DIG_P2_LSB_REG 0x90
#define BME280_DIG_P2_MSB_REG 0x91
#define BME280_DIG_P3_LSB_REG 0x92
#define BME280_DIG_P3_MSB_REG 0x93
#define BME280_DIG_P4_LSB_REG 0x94
#define BME280_DIG_P4_MSB_REG 0x95
#define BME280_DIG_P5_LSB_REG 0x96
#define BME280_DIG_P5_MSB_REG 0x97
#define BME280_DIG_P6_LSB_REG 0x98
#define BME280_DIG_P6_MSB_REG 0x99
#define BME280_DIG_P7_LSB_REG 0x9A
#define BME280_DIG_P7_MSB_REG 0x9B
#define BME280_DIG_P8_LSB_REG 0x9C
#define BME280_DIG_P8_MSB_REG 0x9D
#define BME280_DIG_P9_LSB_REG 0x9E
#define BME280_DIG_P9_MSB_REG 0x9F
#define BME280_DIG_H1_REG 0xA1
#define BME280_CHIP_ID_REG 0xD0 //Chip ID
#define BME280_RST_REG 0xE0		//Softreset Reg
#define BME280_DIG_H2_LSB_REG 0xE1
#define BME280_DIG_H2_MSB_REG 0xE2
#define BME280_DIG_H3_REG 0xE3
#define BME280_DIG_H4_MSB_REG 0xE4
#define BME280_DIG_H4_LSB_REG 0xE5
#define BME280_DIG_H5_MSB_REG 0xE6
#define BME280_DIG_H6_REG 0xE7
#define BME280_CTRL_HUMIDITY_REG 0xF2	 //Ctrl Humidity Reg
#define BME280_STAT_REG 0xF3			 //Status Reg
#define BME280_CTRL_MEAS_REG 0xF4		 //Ctrl Measure Reg
#define BME280_CONFIG_REG 0xF5			 //Configuration Reg
#define BME280_PRESSURE_MSB_REG 0xF7	 //Pressure MSB
#define BME280_PRESSURE_LSB_REG 0xF8	 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG 0xF9	 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG 0xFA	 //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG 0xFB	 //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG 0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG 0xFD	 //Humidity MSB
#define BME280_HUMIDITY_LSB_REG 0xFE	 //Humidity LSB

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct SensorCalibration
{
public:
	bool calibrated = false;
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
};

//This is the main operational class of the driver.

class BME280
{
public:
	SensorCalibration *calibration;
	int32_t t_fine;

	//Constructor generates default BME280_SensorSettings.
	//(over-ride after construction if desired)
	BME280(void);
	//~BME280() = default;

	//Call to apply BME280_SensorSettings.
	//This also gets the SensorCalibration constants
	bool begin(uint8_t address, SensorCalibration *calib);

	void setTempOverSample(uint8_t overSampleAmount);	  //Set the temperature sample mode
	void setHumidityOverSample(uint8_t overSampleAmount); //Set the humidity sample mode
	void setFilter(uint8_t filterSetting);				  //Set the filter
	void setMode(uint8_t mode);

	bool isMeasuring(void); //Returns true while the device is taking measurement

	float readFloatHumidity(void);
	float readTempC(void);

	//The following utilities read and write

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//a chunk of memory into that array.
	void readRegisterRegion(uint8_t *, uint8_t, uint8_t);
	//readRegister reads one register
	uint8_t readRegister(uint8_t);
	//Reads two regs, LSByte then MSByte order, and concatenates them
	//Used for two-byte reads
	int16_t readRegisterInt16(uint8_t offset);
	//Writes a byte;
	void writeRegister(uint8_t, uint8_t);

private:
	uint8_t checkSampleValue(uint8_t userValue); //Checks for valid over sample values

	uint8_t I2CAddress;
};

#endif // End of __BME280_H__ definition check
