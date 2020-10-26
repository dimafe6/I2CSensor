#include "BME280.h"

//Constructor -- Specifies default configuration
BME280::BME280()
{
}

//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored BME280_SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to
//  configure before calling .begin();
//
//****************************************************************************//
bool BME280::begin(uint8_t address, SensorCalibration *calib)
{
	I2CAddress = address;
	calibration = calib;

	Wire.begin();

	//Check communication with IC before anything else
	uint8_t chipID = readRegister(BME280_CHIP_ID_REG); //Should return 0x60 or 0x58
	if (chipID != 0x58 && chipID != 0x60)			   // Is this BMP or BME?
	{
		return false; //This is not BMP nor BME!
	}

	if (!calib->calibrated)
	{
		//Reading all compensation data, range 0x88:A1, 0xE1:E7
		calibration->dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
		calibration->dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
		calibration->dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

		calibration->dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
		calibration->dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
		calibration->dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
		calibration->dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
		calibration->dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
		calibration->dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
		calibration->dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
		calibration->dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
		calibration->dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

		calibration->dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
		calibration->dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
		calibration->dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
		calibration->dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
		calibration->dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
		calibration->dig_H6 = ((int8_t)readRegister(BME280_DIG_H6_REG));

		calibration->calibrated = true;
	}

	setMode(MODE_SLEEP);

	return true;
}

//Set the mode bits in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BME280::setMode(uint8_t mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
}

//Set the filter bits in the config register
//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
void BME280::setFilter(uint8_t filterSetting)
{
	if (filterSetting > 0b111)
		filterSetting = 0; //Error check. Default to filter off

	uint8_t controlData = readRegister(BME280_CONFIG_REG);
	controlData &= ~((1 << 4) | (1 << 3) | (1 << 2)); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2);			  //Align with bits 4/3/2
	writeRegister(BME280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void BME280::setTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~((1 << 7) | (1 << 6) | (1 << 5)); //Clear bits 765
	controlData |= overSampleAmount << 5;			  //Align overSampleAmount to bits 7/6/5
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid over sampling values
void BME280::setHumidityOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_HUMIDITY_REG);
	controlData &= ~((1 << 2) | (1 << 1) | (1 << 0)); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0;			  //Align overSampleAmount to bits 2/1/0
	writeRegister(BME280_CTRL_HUMIDITY_REG, controlData);
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the humidty, pressure, and temp oversample functions
uint8_t BME280::checkSampleValue(uint8_t userValue)
{
	switch (userValue)
	{
	case (0):
		return 0;
		break; //Valid
	case (1):
		return 1;
		break; //Valid
	case (2):
		return 2;
		break; //Valid
	case (4):
		return 3;
		break; //Valid
	case (8):
		return 4;
		break; //Valid
	case (16):
		return 5;
		break; //Valid
	default:
		return 1; //Default to 1x
		break;	  //Good
	}
}

//Check the measuring bit and return true while device is taking measurement
bool BME280::isMeasuring(void)
{
	uint8_t stat = readRegister(BME280_STAT_REG);
	return (stat & (1 << 3)); //If the measuring bit (3) is set, return true
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280::readFloatHumidity(void)
{

	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
	uint8_t buffer[2];
	readRegisterRegion(buffer, BME280_HUMIDITY_MSB_REG, 2);
	int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);

	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration->dig_H4) << 20) - (((int32_t)calibration->dig_H5) * var1)) +
			  ((int32_t)16384)) >>
			 15) *
			(((((((var1 * ((int32_t)calibration->dig_H6)) >> 10) * (((var1 * ((int32_t)calibration->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
				  ((int32_t)calibration->dig_H2) +
			  8192) >>
			 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration->dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1 >> 12) / 1024.0;
}

//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

float BME280::readTempC(void)
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
	uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_TEMPERATURE_MSB_REG, 3);
	int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T >> 3) - ((int32_t)calibration->dig_T1 << 1))) * ((int32_t)calibration->dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calibration->dig_T1)) * ((adc_T >> 4) - ((int32_t)calibration->dig_T1))) >> 12) *
			((int32_t)calibration->dig_T3)) >>
		   14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100;

	return output;
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void BME280::readRegisterRegion(uint8_t *outputPointer, uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.endTransmission();

	// request bytes from slave device
	Wire.requestFrom(I2CAddress, length);
	while ((Wire.available()) && (i < length)) // slave may send less than requested
	{
		c = Wire.read(); // receive a byte as character
		*outputPointer = c;
		outputPointer++;
		i++;
	}
}

uint8_t BME280::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result = 0;
	uint8_t numBytes = 1;

	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.endTransmission();

	Wire.requestFrom(I2CAddress, numBytes);
	while (Wire.available()) // slave may send less than requested
	{
		result = Wire.read(); // receive a byte as a proper uint8_t
	}

	return result;
}

int16_t BME280::readRegisterInt16(uint8_t offset)
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2); //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

	return output;
}

void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	Wire.write(dataToWrite);
	Wire.endTransmission();
}
