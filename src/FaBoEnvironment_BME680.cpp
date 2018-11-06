/**
 @file FaBoEnvironment_BME680.h
 @brief This is a library for the FaBo Environment I2C Brick.

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#include "FaBoEnvironment_BME680.h"

/* combine two 8 bit data's to form a 16 bit data */
#define BME680_CONCAT_BYTES(msb, lsb)	((msb << 8) | lsb)

// BME680 address
int device_addr;

// Temperature OverSampling Parameter
uint8_t _temp_osrs;
// Humidity OverSampling Parameter
uint8_t _hum_osrs;
// Pressure OverSampling Parameter
uint8_t _press_osrs;
// IIR Filter Parameter
uint8_t _iir_filter;
// GAS Heater Parameter
uint16_t _heater_temp = 0;
uint16_t _heater_time = 0;

/* corresponding constants used for the resistance calculation */
// const_array1
uint32_t const_array1[16] = {
  2147483647, 2147483647,
  2147483647, 2147483647,
  2147483647, 2126008810,
  2147483647, 2130303777,
  2147483647, 2147483647,
  2143188679, 2136746228,
  2147483647, 2126008810,
  2147483647, 2147483647
};

// const_array2
uint32_t const_array2[16] = {
  4096000000, 2048000000,
  1024000000, 512000000 ,
  255744255 , 127110228 ,
  64000000  , 32258064  ,
  16016016  , 8000000   ,
  4000000   , 2000000   ,
  1000000   , 500000    ,
  250000    , 125000
};

// BME680 compensation parameters
uint8_t  par_p10, par_h6;
uint16_t par_t1, par_p1, par_h1, par_h2;
int8_t   par_t3, par_p3, par_p6, par_p7, par_h3, par_h4, par_h5, par_h7, par_g1, par_g3;
int16_t  par_t2, par_p2, par_p4, par_p5, par_p8, par_p9, par_g2;

// parameters
int32_t _t_fine;

int32_t _t_temp;
int32_t _t_pressure;
int32_t _t_humidity;
int32_t _t_gas;

uint8_t _res_heat_range;
int8_t _res_heat_val;
int8_t range_sw_err;

/**
 @brief Constructor
*/
FaBoEnvironment::FaBoEnvironment(int addr){
  // set device address
  device_addr = addr;
  Wire.begin();
}

/**
 @brief Begin Device
 @retval true normaly done
 @retval false device error
*/
bool FaBoEnvironment::begin() {
  if ( searchDevice() ) {
    configuration();
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Serch BME680
 * @retval true  : Found
 * @retval false : Not Found
 */
bool FaBoEnvironment::searchDevice()
{
  uint8_t device = readByteI2c(BME680_CHIP_ID_REG);

  if(device == BME680_CHIP_ID){
    return true;
  } else{
    return false;
  }
}

/**
 * @brief Set Config
 */
void FaBoEnvironment::configuration()
{
  // Soft Reset
  writeI2c(BME680_RESET_REG, BME680_INITIATES_RESET_CMD);
  delay(100);
  // Set compensation parameters
  readCalibrationParameter();
}

/**
 * @brief read Calibration parameter
 */
void FaBoEnvironment::readCalibrationParameter()
{
  int8_t rslt;
  uint8_t coeff_array[BME680_COEFF_SIZE] = { 0 };
  uint8_t temp_var = 0; /* Temporary variable */

  readI2c(BME680_CALIB1_START_REG, 25, coeff_array);
  /* Append the second half in the same array */
  readI2c(BME680_CALIB2_START_REG, 16, &coeff_array[25]);

  /* Temperature related coefficients */
  par_t1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG],
    coeff_array[BME680_T1_LSB_REG]));
  par_t2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG],
    coeff_array[BME680_T2_LSB_REG]));
  par_t3 = (int8_t) (coeff_array[BME680_T3_REG]);

  /* Pressure related coefficients */
  par_p1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG],
    coeff_array[BME680_P1_LSB_REG]));
  par_p2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],
    coeff_array[BME680_P2_LSB_REG]));
  par_p3 = (int8_t) coeff_array[BME680_P3_REG];
  par_p4 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG],
    coeff_array[BME680_P4_LSB_REG]));
  par_p5 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],
    coeff_array[BME680_P5_LSB_REG]));
  par_p6 = (int8_t) (coeff_array[BME680_P6_REG]);
  par_p7 = (int8_t) (coeff_array[BME680_P7_REG]);
  par_p8 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],
    coeff_array[BME680_P8_LSB_REG]));
  par_p9 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],
    coeff_array[BME680_P9_LSB_REG]));
  par_p10 = (uint8_t) (coeff_array[BME680_P10_REG]);

  /* Humidity related coefficients */
  par_h1 = (uint16_t) (((uint16_t) coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
    | (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
  par_h2 = (uint16_t) (((uint16_t) coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
    | ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
  par_h3 = (int8_t) coeff_array[BME680_H3_REG];
  par_h4 = (int8_t) coeff_array[BME680_H4_REG];
  par_h5 = (int8_t) coeff_array[BME680_H5_REG];
  par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
  par_h7 = (int8_t) coeff_array[BME680_H7_REG];

  /* Gas heater related coefficients */
  par_g1 = (int8_t) coeff_array[BME680_G1_REG];
  par_g2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_G2_MSB_REG],
    coeff_array[BME680_G2_LSB_REG]));
  par_g3 = (int8_t) coeff_array[BME680_G3_REG];

  uint8_t temp_value;
  /* Other coefficients */
  temp_value = readByteI2c(BME680_RES_HEAT_RANGE_REG);
  _res_heat_range = ((temp_value & BME680_RES_HEAT_RANGE_MASK) >> 4);
  temp_value = readByteI2c(BME680_RES_HEAT_VAL_REG);
  _res_heat_val = (int8_t) temp_value;
  temp_value = readByteI2c(BME680_RANGE_SW_ERR_REG);
  range_sw_err = ((int8_t) temp_value & (int8_t) BME680_RES_ERROR_MASK) >> 4;
}

/**
 * @brief setTempOversampling
 */
void FaBoEnvironment::setParamTempOS(uint8_t osrs_temp)
{
  _temp_osrs = osrs_temp;
}

/**
 * @brief setHumOversampling
 */
void FaBoEnvironment::setParamHumOS(uint8_t osrs_hum)
{
  _hum_osrs = osrs_hum;
}

/**
 * @brief setPressOversampling
 */
void FaBoEnvironment::setParamPressOS(uint8_t osrs_press)
{
  _press_osrs = osrs_press;
}

/**
 * @brief set IIRFilter parameter
 */
void FaBoEnvironment::setParamIIRFilter(uint8_t filter)
{
  _iir_filter = filter;
}

/**
 * @brief set Gas parameter
 */
bool FaBoEnvironment::setParamGasHeater(uint16_t temp,  uint16_t time)
{
  _heater_temp = temp;
  _heater_time = time;

  // GasHeater ON
  if ( _heater_temp!=0 && _heater_time!=0 ) {
    // check temperature to min/max
    if (_heater_temp < 200) _heater_temp = 200;
    else if (_heater_temp > 400) _heater_temp = 400;
  }
}

/**
 * @brief Set Mode
 */
void FaBoEnvironment::setMode(uint8_t mode)
{
	uint8_t ctrl_meas_val;
	uint8_t pow_mode = 0;
  // Serial.print("set mode:");
  // Serial.println(mode,BIN);

  // Check SLEEP_MODE
	do {
		ctrl_meas_val = readByteI2c(BME680_CTRL_MEAS_REG);

    // Serial.print("READ:");Serial.println(ctrl_meas_val,BIN);
		/* Put to sleep before changing mode */
		pow_mode = ctrl_meas_val & BME680_MODE_MASK;

		if (pow_mode != BME680_SLEEP_MODE) {
			ctrl_meas_val = ctrl_meas_val & (~BME680_MODE_MASK); /* Set to sleep */
      // Serial.print("SET:");Serial.println(ctrl_meas_val,BIN);

			writeI2c(BME680_CTRL_MEAS_REG, ctrl_meas_val);
			delay(50);
		}
	} while (pow_mode != BME680_SLEEP_MODE);

	/* set mode */
	if (mode != BME680_SLEEP_MODE) {
		ctrl_meas_val = ((ctrl_meas_val & (~BME680_MODE_MASK)) | mode);
		writeI2c(BME680_CTRL_MEAS_REG, ctrl_meas_val);
	}
}

/**
 * @brief set Sensor Data(Temperature, Pressure, Humidity)
 */
void FaBoEnvironment::setSensor()
{
  /* Set IIR Filter */
  // Read config
  uint8_t read_config = (readByteI2c(BME680_CONFIG_REG) & (~BME680_CONFIG_IIR_FILTER_MASK));
  read_config |= _iir_filter;
  // Set config
  writeI2c(BME680_CONFIG_REG, read_config);

  /* Set Over Sampling(Temperature, Pressure) */
  // Read CTRL_MEAS REG
  uint8_t read_mode = (readByteI2c(BME680_CTRL_MEAS_REG) & BME680_MODE_MASK);

  /* Set CTRL_MEAS */
  uint8_t ctrl_meas = read_mode
                      | (_temp_osrs<<BME680_OSRS_TEMP_BIT_SHIFT)
                      | (_press_osrs<<BME680_OSRS_PRESS_BIT_SHIFT);
  writeI2c(BME680_CTRL_MEAS_REG, ctrl_meas);

  /* Set Over Sampling(Humidity) */
  writeI2c(BME680_CTRL_HUM_REG, _hum_osrs<<BME680_OSRS_HUM_BIT_SHIFT);

}

/**
 * @brief set Gas Data
 */
bool FaBoEnvironment::setGasHeater()
{
  uint8_t gas_register = readByteI2c(BME680_CTRL_GAS1_REG);

  // GasHeater OFF
  if ( _heater_temp==0 || _heater_time==0 ) {
    // Serial.println("Heater OFF");
    // Heater OFF
    writeI2c(BME680_CTRL_GAS0_REG, BME680_CTRL_GAS0_HEATER_OFF);
    // Gas measurements OFF
    writeI2c(BME680_CTRL_GAS1_REG, (gas_register&BME680_CTRL_RUN_GAS_MASK));
  } else {

    // _res_heat_range = (readByteI2c(BME680_RES_HEAT_RANGE_REG) & BME680_RES_HEAT_RANGE_MASK) >> 4;
    // _res_heat_val = readByteI2c(BME680_RES_HEAT_VAL_REG);

    int32_t var1 = (((int32_t)_t_temp*par_g3)/1000)*256;
    int32_t var2 = (par_g1+784)*(((((par_g2+154009)*_heater_temp*5)/100)+3276800)/10);
    int32_t var3 = var1 + (var2 / 2);
    int32_t var4 = (var3 / (_res_heat_range+4));
    int32_t var5 = (131 * _res_heat_val) + 65536;
    int32_t heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
    uint8_t res_heat_x = (uint8_t) ((heatr_res_x100 + 50) / 100);

    // 0x5A
    writeI2c(BME680_RES_HEAT_0_REG, res_heat_x);

    uint8_t factor = 0;
    uint8_t wait;
    uint8_t time = _heater_time;

    if (time >= BME680_GAS_WAIT_MAX_TIME) {
      // MAX Value 0xFF-> factor:0xB0(64) + value:0x3F
      wait = 0xFF;
    } else {
      while (time > 0x3F) {
        time /= 4;
        factor += 1;
      }
      wait = (uint8_t) (time | (factor << BME680_GAS_WAIT_FACTOR_BIT_SHIFT));
    }

    writeI2c(BME680_GAS_WAIT_0_REG, wait);
    writeI2c(BME680_CTRL_GAS1_REG, BME680_CTRL_GAS1_RUN_GAS);
  }
}

/**
 * @brief read Sensor Value
 * @retval true: data found
 * @retval false: no data found
 */
bool FaBoEnvironment::readSensors()
{
  // check status
  if ((readByteI2c(BME680_EAS_STATUS_REG)&BME680_EAS_STATUS_MEASURING)!=0){
    return false;
  };

  // set sleep mode
  setMode(BME680_SLEEP_MODE);
  // set sensor data
  setGasHeater();
  setSensor();
  setMode(BME680_FORCED_MODE);
  // read delay time
  uint16_t delay_time = getOversamplingDuration();

  // New data Check
  do{
    delay(delay_time * 2);
  }while ((readByteI2c(BME680_EAS_STATUS_REG)&BME680_EAS_STATUS_NEW_DATA)!=0);

  /* calculation sensor data */

  // temperature
  uint8_t raw_temp_data[3];
  uint32_t raw_temp;
  int32_t var1 = 0, var2 = 0, var3 = 0, var4 = 0, var5 = 0, var6 = 0;

  readI2c(BME680_TEMP_MSB_REG, 3, raw_temp_data);
  raw_temp = (((uint32_t) raw_temp_data[0] << 16 | (uint32_t) raw_temp_data[1] << 8 | raw_temp_data[2]) >> 4);

  // set temperature in DegC
  var1 = ((int32_t) raw_temp >> 3) - ((int32_t)par_t1 << 1);
  var2 = (var1 * (int32_t)par_t2) >> 11;
  var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t) par_t3 << 4)) >> 14;
  _t_fine = var2 + var3;
  _t_temp = (_t_fine * 5 + 128) >> 8;

  // Pressure
  uint8_t raw_pressure_data[3];
  uint32_t raw_pressure;
  readI2c(BME680_PRESS_MSB_REG, 3, raw_pressure_data);
  raw_pressure = (uint32_t) (((uint32_t) raw_pressure_data[0] << 16
                 | (uint32_t) raw_pressure_data[1] << 8 | raw_pressure_data[2]) >> 4);

  // set Pascal
  int32_t pascal = 0;
  var1 = (((int32_t) _t_fine) >> 1) - 64000;
  var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) par_p6) >> 2;
  var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
  var2 = (var2 >> 2) + ((int32_t) par_p4 << 16);
  var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t) par_p3 << 5)) >> 3)
         + (((int32_t) par_p2 * var1) >> 1);
  var1 = var1 >> 18;
  var1 = ((32768 + var1) * (int32_t) par_p1) >> 15;
  pascal = 1048576 - raw_pressure;
  pascal = (int32_t)((pascal - (var2 >> 12)) * ((uint32_t)3125));

  // check over flow
  if(pascal >= 0x40000000){
    pascal = (( pascal / (uint32_t) var1) << 1);
  } else {
    pascal = ((pascal << 1) / (uint32_t) var1);
  }

  var1 = ((int32_t) par_p9 * (int32_t) (((pascal >> 3)
         * (pascal >> 3)) >> 13)) >> 12;
  var2 = ((int32_t)(pascal >> 2) * (int32_t) par_p8) >> 13;
  var3 = ((int32_t)(pascal >> 8) * (int32_t)(pascal >> 8)
         * (int32_t)(pascal >> 8) * (int32_t)par_p10) >> 17;
  pascal = (int32_t)(pascal) + ((var1 + var2 + var3
           + ((int32_t)par_p7 << 7)) >> 4);
  _t_pressure = pascal;

  // set humidity
  uint8_t raw_humi_data[2];
  uint16_t raw_Humidity;
  readI2c(BME680_HUM_MSB_REG, 2, raw_humi_data);
  raw_Humidity = (((uint16_t) raw_humi_data[0] << 8 | raw_humi_data[1]) );

  int32_t temperature;
  int32_t humidity;

  temperature = (((int32_t) _t_fine * 5) + 128) >> 8;
  var1 = (int32_t) raw_Humidity  - ((int32_t) ((int32_t)par_h1 << 4)) - (((temperature * (int32_t) par_h3) / ((int32_t)100)) >> 1);
  var2 = ((int32_t)par_h2 * (((temperature * (int32_t)par_h4)
         / ((int32_t)100)) + (((temperature * ((temperature * (int32_t)par_h5)
         / ((int32_t)100))) >> 6) / ((int32_t)100)) + (int32_t)(1 << 14))) >> 10;
  var3 = var1 * var2;
  var4 = ((((int32_t)par_h6) << 7) + ((temperature * (int32_t) par_h7)
         / ((int32_t)100))) >> 4;
  var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
  var6 = (var4 * var5) >> 1;

  humidity = (var3 + var6) >> 12;

  // check for over- and under-flow
  if (humidity > 102400) {
   humidity = 102400;
  } else if(humidity < 0) {
   humidity = 0;
  }
  _t_humidity = humidity;

  // GAS
  uint8_t raw_gas_data[2];
  uint16_t raw_gas;
  readI2c(BME680_GAS_R_MSB_REG, 2, raw_gas_data);

  raw_gas = (((uint16_t) raw_gas_data[0] << 2 | (0xC0 & raw_gas_data[1]) >> 6) );

  uint8_t gasRange = readByteI2c(BME680_GAS_R_LSB_REG) & 0x0F;
  int64_t gas_var1 = (int64_t)((1340+(5*(int64_t)range_sw_err))*((int64_t) const_array1[gasRange])) >> 16;
  uint64_t gas_var2 = (((int64_t)((int64_t)raw_gas<<15)-(int64_t)(16777216))+gas_var1);
  int64_t gas_var3 = (((int64_t) const_array2[gasRange] * (int64_t) gas_var1) >> 9);
  uint32_t gas_res = (uint32_t) ((gas_var3 + ((int64_t) gas_var2 >> 1)) / (int64_t) gas_var2);
  _t_gas = gas_res;

  return true;
}

/*
 * @brief Get OverSampling Duration.
 */
uint16_t FaBoEnvironment::getOversamplingDuration()
{
	uint32_t calc_duration;
	uint32_t meas_cycles;
  uint16_t duration;
  // Over Sampling List
	uint8_t dur_array[6] = {0, 1, 2, 4, 8, 16};

	meas_cycles = dur_array[_temp_osrs];    // temp
	meas_cycles += dur_array[_hum_osrs];    // humidity
	meas_cycles += dur_array[_press_osrs];  // pressure

	/* TPH measurement duration */
	calc_duration = meas_cycles * 1963;
	calc_duration += 477 * 4;  /* TPH switching duration */
	calc_duration += 477 * 5;  /* Gas measurement duration */
	calc_duration += 500;      /* Get it to the closest whole number.*/
	calc_duration /= 1000;     /* Convert to ms */
	calc_duration += 1;        /* Wake up duration of 1ms */

	duration = (uint16_t) calc_duration;

	/* Get the gas duration only set heater_time */
	if (_heater_time>0) {
		/* Add heater time*/
		duration += _heater_time;
	}
  return duration;
}

/*
 * @brief read Temperature
 * @return uint32_t : Temperature (Degree Celsius)
 */
float FaBoEnvironment::readTemperature()
{
  return (float)_t_temp / 100;
}

/*
 * @brief read Pressure
 * @return uint32_t : Pressure (pascal)
 */
float FaBoEnvironment::readPressure()
{
  return (float)_t_pressure;
}

/*
 * @brief read Humidity
 * @return float : Humidity (Relative Humidity:%)
 */
float FaBoEnvironment::readHumidity()
{
  return (float)_t_humidity / 1024;
}

/*
 * @brief read Gas Resistance
 * @return uint32_t : Gas Rasistance(Ohm)
 */
uint32_t FaBoEnvironment::readGasResistance()
{
  return _t_gas;
}

float FaBoEnvironment::readAltitude()
{
  float altitude;
  altitude = (float)145366.45f*(1.0f - powf((_t_pressure/100/1013.25f), 0.190284f))*0.3048;

  return altitude;
}

/**
 * @brief Write I2C Data
 * @param [in] register_addr : Write Register Address
 * @param [in] value  : Write Data
 */
void FaBoEnvironment::writeI2c(uint8_t register_addr, uint8_t value) {
  // Serial.print("[W]\tI2C 0x");
  // Serial.print(register_addr,HEX);
  // Serial.print(" : 0x");
  // Serial.println(value, HEX);
  Wire.beginTransmission(device_addr);
  Wire.write(register_addr);
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * @brief Read I2C Data
 * @param [in] register_addr : register address
 * @param [in] num   : Data Length
 * @param [out] *buf : Read Data
 */
void FaBoEnvironment::readI2c(uint8_t register_addr, uint8_t num, uint8_t *buf) {
  // Serial.print("[R]\tI2C 0x");
  // Serial.print(register_addr,HEX);
  // Serial.print(" : ");
  Wire.beginTransmission(device_addr);
  Wire.write(register_addr);
  Wire.endTransmission();

  //Wire.beginTransmission(DEVICE_ADDR);
  Wire.requestFrom((int)device_addr, (int)num);

  int i = 0;
  while (Wire.available())
  {
    buf[i] = Wire.read();
    // Serial.print("0x");
    // Serial.print(buf[i],HEX);
    // Serial.print(", ");
    i++;
  }
  // Serial.println("");
}

/**
 * @brief Read I2C Byte
 * @param [in] register_addr : register address
 * @return uint8_t : Read byte
 */
uint8_t FaBoEnvironment::readByteI2c(uint8_t register_addr) {
  Wire.beginTransmission(device_addr);
  Wire.write(register_addr);
  Wire.endTransmission();
  Wire.requestFrom(device_addr, 1);
  uint8_t value = Wire.read();
  // Serial.print("[R]\tI2C 0x");
  // Serial.print(register_addr,HEX);
  // Serial.print(" : 0x");
  // Serial.println(value,HEX);
  return value;
}
