/**
 @file FaBoEnvironment_BME680.cpp
 @brief This is a library for the FaBo Environment I2C Brick.

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#ifndef FABOENVIRONMENT_BME680_H
#define FABOENVIRONMENT_BME680_H

#include "Arduino.h"
#include "Wire.h"

/** BME680 Slave Address register */
#define BME680_SLAVE_ADDRESS_DEFAULT 0x76

/** Who_am_i register */
#define BME680_CHIP_ID_REG	0xD0
/** BME680 Chip ID **/
#define BME680_CHIP_ID 			0x61

// BME680 Register
#define BME680_STATUS_REG					0x73
#define BME680_RESET_REG					0xE0
#define BME680_CONFIG_REG					0x75
#define BME680_CTRL_MEAS_REG			0x74
#define BME680_CTRL_HUM_REG				0x72
#define BME680_CTRL_GAS1_REG			0x71
#define BME680_CTRL_GAS0_REG			0x70
#define BME680_GAS_WAIT_0_REG			0x64 // gas_wait_x :0x64-0x6D
#define BME680_RES_HEAT_0_REG			0x5A // res_heat_x :0x5A-0x63
#define BME680_IDAC_HEAT_0_REG		0x50 // idac_heat_x:0x50-0x59
#define BME680_GAS_R_LSB_REG			0x2B
#define BME680_GAS_R_MSB_REG			0x2A
#define BME680_HUM_LSB_REG				0x26
#define BME680_HUM_MSB_REG				0x25
#define BME680_TEMP_XLSB_REG			0x24
#define BME680_TEMP_LSB_REG				0x23
#define BME680_TEMP_MSB_REG				0x22
#define BME680_PRESS_XLSB_REG			0x21
#define BME680_PRESS_LSB_REG			0x20
#define BME680_PRESS_MSB_REG			0x1F
#define BME680_EAS_STATUS_REG			0x1D
#define BME680_RANGE_SW_ERR_REG		0x04
#define BME680_RES_HEAT_RANGE_REG	0x02
#define BME680_RES_HEAT_VAL_REG		0x00

#define BME680_CALIB1_START_REG		0x89
#define BME680_CALIB2_START_REG		0xE1

#define BME680_PAR_G3_REG					0xEE
#define BME680_PAR_G1_REG					0xED
#define BME680_PAR_G2_MSB_REG			0xEC
#define BME680_PAR_G2_LSB_REG			0xEB

// IIR Fillter Control (0x75:2-4bit)
#define BME680_IIR_FILTER_0		0b00000000
#define BME680_IIR_FILTER_1		0b00000100
#define BME680_IIR_FILTER_3		0b00001000
#define BME680_IIR_FILTER_7		0b00001100
#define BME680_IIR_FILTER_15	0b00010000
#define BME680_IIR_FILTER_31	0b00010100
#define BME680_IIR_FILTER_63	0b00011000
#define BME680_IIR_FILTER_127	0b00011100

#define BME680_CONFIG_IIR_FILTER_MASK	0b00011100

#define BME680_OSRS_TEMP_MASK		0b11100000
#define BME680_OSRS_PRESS_MASK	0b00011100
#define BME680_MODE_MASK				0b00000011
#define BME680_OSRS_HUM_MASK		0b00000111

// CTRL_GAS0 Heater Off
#define BME680_CTRL_GAS0_HEATER_OFF		0b00001000
#define BME680_CTRL_GAS0_HEATER_ON		0b00000000

// CTRL_GAS1 run_gas
#define BME680_CTRL_GAS1_RUN_GAS			0b00010000
// CTRL_GAS1 run_gas MASK
#define BME680_CTRL_RUN_GAS_MASK			0b11101111

// IIR_FILTER MASK
#define BME680_CONFIG_IIR_FILTER_MASK	0b11100011
#define BME680_RES_HEAT_RANGE_MASK		0b00110000
#define BME680_RES_ERROR_MASK					0b11110000

// Gas sensor wait time multiplication factor (6-7bit)
#define BME680_GAS_WAIT_FACTOR_BIT_SHIFT		6

// Max Time (factor:64 * MaxValue:63 -> 4032)
#define BME680_GAS_WAIT_MAX_TIME		0xFC0

#define BME680_OSRS_SKIP	0b000
#define BME680_OSRS_1			0b001 //1
#define BME680_OSRS_2			0b010	//2
#define BME680_OSRS_4			0b011	//3
#define BME680_OSRS_8			0b100	//4
#define BME680_OSRS_16		0b101	//5

#define BME680_SLEEP_MODE		0b00
#define BME680_FORCED_MODE	0b01

// Over Sampling Temperature Bit point(0x74:5-7bit)
#define BME680_OSRS_TEMP_BIT_SHIFT	5
// Over Sampling Pressure Bit point (0x74:5-7bit)
#define BME680_OSRS_PRESS_BIT_SHIFT	2
// Over Sampling Humidity Bit point (0x74:0-2bit)
#define BME680_OSRS_HUM_BIT_SHIFT		0


#define BME680_COEFF_SIZE 41

/* Calibration Parameters */
#define BME680_T2_LSB_REG	1
#define BME680_T2_MSB_REG	2
#define BME680_T3_REG			3
#define BME680_P1_LSB_REG	5
#define BME680_P1_MSB_REG	6
#define BME680_P2_LSB_REG	7
#define BME680_P2_MSB_REG	8
#define BME680_P3_REG			9
#define BME680_P4_LSB_REG	11
#define BME680_P4_MSB_REG	12
#define BME680_P5_LSB_REG	13
#define BME680_P5_MSB_REG	14
#define BME680_P7_REG			15
#define BME680_P6_REG			16
#define BME680_P8_LSB_REG	19
#define BME680_P8_MSB_REG	20
#define BME680_P9_LSB_REG	21
#define BME680_P9_MSB_REG	22
#define BME680_P10_REG		23
#define BME680_H2_MSB_REG	25
#define BME680_H2_LSB_REG	26
#define BME680_H1_LSB_REG	26
#define BME680_H1_MSB_REG	27
#define BME680_H3_REG			28
#define BME680_H4_REG			29
#define BME680_H5_REG			30
#define BME680_H6_REG			31
#define BME680_H7_REG			32
#define BME680_T1_LSB_REG	33
#define BME680_T1_MSB_REG	34
#define BME680_G2_LSB_REG	35
#define BME680_G2_MSB_REG	36
#define BME680_G1_REG			37
#define BME680_G3_REG			38

#define BME680_HUM_REG_SHIFT_VAL	4
#define	BME680_BIT_H1_DATA_MSK	  0x0F

#define BME680_RES_HEAT_RANGE_MASK	0b00110000

#define BME680_EAS_STATUS_MEASURING			0b00100000
#define BME680_EAS_STATUS_NEW_DATA			0b10000000

// Soft Reset COMMAND
#define BME680_INITIATES_RESET_CMD	0xB6

/**
 * @class FaBo9AxisBMX
 * @brief BMX055 Control
 */
class FaBoEnvironment
{
public:
	FaBoEnvironment(int addr = BME680_SLAVE_ADDRESS_DEFAULT);
	bool begin(void);
	bool searchDevice(void);
	void configuration(void);
	void readCalibrationParameter(void);

	// set parameters
	void setParamTempOS(uint8_t osrs_temp);
	void setParamPressOS(uint8_t osrs_press);
	void setParamHumOS(uint8_t osrs_hum);
	void setParamIIRFilter(uint8_t filter);
	bool setParamGasHeater(uint16_t target_temp,  uint16_t time);

	void setSensor(void);
	bool setGasHeater(void);
	void setMode(uint8_t mode);

	uint16_t getOversamplingDuration();

	bool readSensors(void);
	float readTemperature(void);
	float readPressure(void);
	float readHumidity(void);
	uint32_t readGasResistance(void);
	float readAltitude(void);

private:
	void writeI2c(uint8_t register_addr, uint8_t value);
	void readI2c(uint8_t register_addr, uint8_t num, uint8_t *buf);
	uint8_t readByteI2c(uint8_t register_addr);
};

#endif
