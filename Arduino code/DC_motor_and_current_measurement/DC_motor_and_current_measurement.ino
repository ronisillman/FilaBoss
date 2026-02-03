#include <Wire.h>
#include <Adafruit_INA219.h>

//two current sensors
Adafruit_INA219 ina219_spool(0x40);
Adafruit_INA219 ina219_roller(0x41);

//motor pins
const int motorRollerPin = 3;
const int motorSpoolPin = 11;

//motor speeds
int speed_Roller = 15;
int speed_Spool = 30;

void setup() {
    Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

  // Initialize INA219 at 0x40
  if (!ina219_spool.begin()) {
    Serial.println("Failed to find INA219 at address 0x40");
    while (1) {
      delay(10);
    }
  }

  // Initialize INA219 at 0x41
  if (!ina219_roller.begin()) {
    Serial.println("Failed to find INA219 at address 0x41");
    while (1) {
      delay(10);
    }
  }

  pinMode(motorRollerPin, OUTPUT);
  pinMode(motorSpoolPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Beginning code.");

}

void loop() {
  analogWrite(motorRollerPin, speed_Roller);
  analogWrite(motorSpoolPin, speed_Spool);

  float current_spool_mA = ina219_spool.getCurrent_mA();
  float current_roller_mA = ina219_roller.getCurrent_mA();

  Serial.print("Current Spool motor (0x40): ");
  Serial.print(current_spool_mA);
  Serial.println(" mA");

  Serial.print("Current Roller motor (0x41): ");
  Serial.print(current_roller_mA);
  Serial.println(" mA");

  Serial.println();
  delay(1000);
}
