#include <Wire.h>
#include "DFRobot_AHT20.h"
#include "DFRobot_ENS160.h"
#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <LiquidCrystal.h>

// LCD pins
const int rs = 21, en = 19, d4 = 5, d5 = 8, d6 = 7, d7 = 15;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

DFRobot_AHT20 aht20;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
MAX30205 max30205;
MAX30105 particleSensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize LCD
  lcd.begin(16, 2);

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
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(aht20.getTemperature_C());
    lcd.println(" 'C");
    lcd.setCursor(0, 1); //Go to second line
    lcd.print("Humi: ");
    lcd.print(aht20.getHumidity_RH());
    lcd.println(" %RH");
    delay(3000);
  }

  // ENS160
  lcd.clear();
  lcd.print("AQI: ");
  lcd.println(ens160.getAQI());
  lcd.setCursor(0, 1); //Go to second line
  lcd.print("TVOC: ");
  lcd.print(ens160.getTVOC());
  lcd.println(" ppb");
  delay(3000);

  // MAX30205 and eCO2 of ENS160
  lcd.clear();
  lcd.print("eCO2: ");
  lcd.print(ens160.getECO2());
  lcd.println(" ppm");
  lcd.setCursor(0, 1); //Go to second line
  float temp = max30205.getTemperature();
  lcd.print("Body: ");
  lcd.print(temp, 2);
  lcd.println(" 'C");
  delay(3000);

  // MAX30105
  lcd.clear();
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

  lcd.print("HR: ");
  lcd.println(heartRate);
  lcd.setCursor(0, 1); //Go to second line
  lcd.print("SPO2: ");
  lcd.println(spo2);
  delay(3000);
}
