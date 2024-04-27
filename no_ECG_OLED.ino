#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "DFRobot_AHT20.h"
#include "DFRobot_ENS160.h"
#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

DFRobot_AHT20 aht20;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
MAX30205 max30205;
MAX30105 particleSensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {}
  }
  display.display();
  delay(2000);
  display.clearDisplay();

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
  // Clear display before updating
  display.clearDisplay();

  // Print sensor data to the OLED display
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // AHT20
  if (aht20.startMeasurementReady(/* crcEn = */true)) {
    display.setCursor(0, 0); // Set cursor position
    display.print("Temperature: ");
    display.print(aht20.getTemperature_C());
    display.println(" 'C");

    display.print("Humidity: ");
    display.print(aht20.getHumidity_RH());
    display.println(" %RH");
  }

  // ENS160
  display.print("Air quality index: ");
  display.println(ens160.getAQI());

  display.print("TVOC: ");
  display.print(ens160.getTVOC());
  display.println(" ppb");

  display.print("eCO2: ");
  display.print(ens160.getECO2());
  display.println(" ppm");

  // MAX30205
  display.print("Body temp: ");
  display.print(max30205.getTemperature());
  display.println(" 'C");

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

  display.print("Heart rate: ");
  display.print(heartRate);
  display.println(" BPM");

  display.print("SPO2: ");
  display.print(spo2);
  display.println(" %");

  display.display(); // Show the data on the OLED screen
  delay(1000);
}
