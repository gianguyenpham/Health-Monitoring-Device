#include <Wire.h>
#include "DFRobot_AHT20.h"
#include "DFRobot_ENS160.h"
#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"

DFRobot_AHT20 aht20;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
MAX30205 max30205;
MAX30105 particleSensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize AHT20 sensor
  uint8_t status;
  while ((status = aht20.begin()) != 0) {
    Serial.print("AHT20 sensor initialization failed. Error status: ");
    Serial.println(status);
    delay(1000);
  }

  // Initialize ENS160 sensor
  while (NO_ERR != ens160.begin()) {
    Serial.println("Communication with ENS160 sensor failed, please check connection");
    delay(3000);
  }
  Serial.println("ENS160 Begin ok!");

  ens160.setPWRMode(ENS160_STANDARD_MODE);
  ens160.setTempAndHum(25.0, 50.0);

  // Initialize MAX30205 sensor
  while (!max30205.scanAvailableSensors()) {
    Serial.println("Couldn't find the MAX30205 temperature sensor, please connect the sensor.");
    delay(30000);
  }
  max30205.begin();

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void loop() {
  // AHT20
  if (aht20.startMeasurementReady(/* crcEn = */true)) {
    Serial.print("AHT20 - Temperature: ");
    Serial.print(aht20.getTemperature_C());
    Serial.println(" 'C");

    Serial.print("AHT20 - Humidity: ");
    Serial.print(aht20.getHumidity_RH());
    Serial.println(" %RH");
  }

  // ENS160
  Serial.print("ENS160 - Air quality index: ");
  Serial.println(ens160.getAQI());

  Serial.print("ENS160 - Concentration of total volatile organic compounds: ");
  Serial.print(ens160.getTVOC());
  Serial.println(" ppb");

  Serial.print("ENS160 - Carbon dioxide equivalent concentration: ");
  Serial.print(ens160.getECO2());
  Serial.println(" ppm");

  // MAX30205
  float temp = max30205.getTemperature();
  Serial.print("MAX30205 - Body temperature: ");
  Serial.print(temp, 2);
  Serial.println(" 'C");

  // MAX30105
  int32_t bufferLength = 100;
  uint32_t irBuffer[bufferLength];
  uint32_t redBuffer[bufferLength];

  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  int32_t spo2;
  int8_t validSPO2;
  int32_t heartRate;
  int8_t validHeartRate;

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  Serial.print("MAX30105 - Heart rate: ");
  Serial.print(heartRate);
  Serial.println(" BPM");

  Serial.print("MAX30105 - SPO2: ");
  Serial.print(spo2);
  Serial.println(" %");

  Serial.println();
  delay(1000);
}
