/**
 @file readEnvironment.ino
 @brief This is an Example for the FaBo Environment I2C Brick.

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/
#include <FaBoEnvironment_BME680.h>

FaBoEnvironment faboEnv;

void setup() {
  Serial.begin(115200);
  // init BME680
  while(!faboEnv.begin()){
    Serial.println("BME680 Not Found");
    delay(5000);
  }
  Serial.println("I am BME680");

  // BME680 Setup OverSampling
  faboEnv.setParamTempOS(BME680_OSRS_8);
  faboEnv.setParamHumOS(BME680_OSRS_2);
  faboEnv.setParamPressOS(BME680_OSRS_4);
  // BME680 Setup IIR Filter
  faboEnv.setParamIIRFilter(BME680_IIR_FILTER_3);
  // BME680 Setup GasHeater
  faboEnv.setParamGasHeater(320, 150);
  
}

void loop() {
  // Read Sensor
  if(faboEnv.readSensors()){
  
    float temp = faboEnv.readTemperature();
    float pressure = faboEnv.readPressure()/100; // Pa -> hPa
    float humidity = faboEnv.readHumidity();
    long resistance = faboEnv.readGasResistance();
    float altitude = faboEnv.readAltitude();
    
    Serial.print("Time:");
    Serial.print((float)millis()/1000);
    Serial.println(" s");
    
    Serial.print("Temperature = "); 
    Serial.print( temp, 2); 
    Serial.println(" C");     // degrees Celsius
    
    Serial.print("Pressure = "); 
    Serial.print(pressure, 2);
    Serial.println(" hPa");   // hPa
  
    Serial.print("Altitude = ");
    Serial.print(altitude, 2);
    Serial.println("m");
    
    Serial.print("Humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %");     // Relative Humidity (%)
    
    Serial.print("Gas Resistance = "); 
    Serial.print(resistance);  
    Serial.println(" Ohm");   // Ohm
    Serial.println("");
  }
}
