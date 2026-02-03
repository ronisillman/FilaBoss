#include <Wire.h>
#include <Adafruit_INA219.h>

// Create two INA219 objects with different I2C addresses
Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x41);

void setup(void)
{
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

  // Initialize INA219 at 0x40
  if (!ina219_1.begin()) {
    Serial.println("Failed to find INA219 at address 0x40");
    while (1) {
      delay(10);
    }
  }

  // Initialize INA219 at 0x41
  if (!ina219_2.begin()) {
    Serial.println("Failed to find INA219 at address 0x41");
    while (1) {
      delay(10);
    }
  }

  // Optional: set calibration if you want better resolution
  // ina219_1.setCalibration_32V_1A();
  // ina219_2.setCalibration_32V_1A();

  Serial.println("Measuring current from two INA219 sensors...");
}

void loop(void)
{
  float current1_mA = ina219_1.getCurrent_mA();
  float current2_mA = ina219_2.getCurrent_mA();

  Serial.print("Current Sensor 1 (0x40): ");
  Serial.print(current1_mA);
  Serial.println(" mA");

  Serial.print("Current Sensor 2 (0x41): ");
  Serial.print(current2_mA);
  Serial.println(" mA");

  Serial.println();
  delay(1000);
}