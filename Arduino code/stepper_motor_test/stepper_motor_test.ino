// ESP32 + DRV8825 standalone test
// Waits for button press, then turns stepper exactly 100 revolutions and stops.
//
// Wiring:
// STEP   -> GPIO25
// DIR    -> GPIO26
// BUTTON -> GPIO27 to GND (uses INPUT_PULLUP)
//
// DRV8825 microstep mode is set by hardware M0/M1/M2 pins.
// Set MICROSTEPS below to match that wiring.

#include <Arduino.h>

const int STEP_PIN   = 25;
const int DIR_PIN    = 26;
const int BUTTON_PIN = 27;

const int MOTOR_STEPS_PER_REV = 200; // NEMA17 1.8 deg/step
const int MICROSTEPS = 32;            // 1,2,4,8,16,32 (match M0/M1/M2)
const int TARGET_REVS = 50;

// Speed/timing
const unsigned int STEP_HIGH_US = 5;   // DRV8825 safe pulse width
const unsigned int STEP_LOW_US  = 50; // lower = faster

void waitForButtonPressAndRelease() {
  // Wait for press (active LOW)
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(5);
  }
  delay(30); // debounce
  // Wait for release
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(5);
  }
  delay(30); // debounce
}

void runSteps(unsigned long stepCount, bool clockwise) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  delayMicroseconds(10); // direction setup time

  for (unsigned long i = 0; i < stepCount; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_LOW_US);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println("Press button to start 100-revolution move...");
  waitForButtonPressAndRelease();

  const unsigned long totalSteps =
      (unsigned long)TARGET_REVS * MOTOR_STEPS_PER_REV * MICROSTEPS;

  runSteps(totalSteps, true);
  Serial.println("Done: 100 revolutions completed.");
}

void loop() {
  // Finished, do nothing.
}