#include <Wire.h>
#include "DFRobot_AHT20.h"
#include "DFRobot_ENS160.h"
#include "Protocentral_MAX30205.h"

// Initialize sensor objects
DFRobot_AHT20 aht20;
DFRobot_ENS160_I2C ENS160(&Wire, 0x53);
MAX30205 tempSensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize AHT20 sensor
  uint8_t status;
  while((status = aht20.begin()) != 0){
    Serial.print("AHT2x sensor initialization failed. error status : ");
    Serial.println(status);
    delay(1000);
  }

  // Initialize ENS160 sensor
  while( NO_ERR != ENS160.begin() ){
    Serial.println("Communication with ENS160 device failed, please check connection");
    delay(3000);
  }
  Serial.println("ENS160 Begin ok!");

  ENS160.setPWRMode(ENS160_STANDARD_MODE);
  ENS160.setTempAndHum(25.0, 50.0);

  // Initialize MAX30205 sensor
  while(!tempSensor.scanAvailableSensors()){
    Serial.println("Couldn't find the temperature sensor, please connect the sensor." );
    delay(30000);
  }
  tempSensor.begin();
}

void loop() {
  // Read and print AHT20 temperature and humidity data
  if(aht20.startMeasurementReady(/* crcEn = */true)){
    Serial.print("AHT2x - Temperature: ");
    Serial.print(aht20.getTemperature_C());
    Serial.println(" 'C");
    Serial.print("AHT2x - Humidity: ");
    Serial.print(aht20.getHumidity_RH());
    Serial.println(" %RH");
  }

  // Read and print ENS160 air quality data
  uint8_t Status = ENS160.getENS160Status();
  //Serial.print("ENS160 - Sensor operating status: ");
  //Serial.println(Status);
  
  uint8_t AQI = ENS160.getAQI();
  Serial.print("ENS160 - Air quality index: ");
  Serial.println(AQI);
  
  uint16_t TVOC = ENS160.getTVOC();
  Serial.print("ENS160 - Concentration of total volatile organic compounds: ");
  Serial.print(TVOC);
  Serial.println(" ppb");
  
  uint16_t ECO2 = ENS160.getECO2();
  Serial.print("ENS160 - Carbon dioxide equivalent concentration: ");
  Serial.print(ECO2);
  Serial.println(" ppm");

  // Read and print MAX30205 body temperature data
  float temp = tempSensor.getTemperature();
  Serial.print("MAX30205 - Body temperature: ");
  Serial.print(temp, 2);
  Serial.println(" 'C");

  Serial.println();
  delay(1000);
}
