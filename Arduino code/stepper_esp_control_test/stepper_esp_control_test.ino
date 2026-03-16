// Standalone test for stepperControl() logic from Winding_line_control
// Target: ESP32 (ESP-WROOM-32) + DRV8825 + NEMA17 (200 steps/rev)
// Pins: STEP=GPIO25, DIR=GPIO26
//
// DRV8825 full-step mode in hardware:
// M0=LOW, M1=LOW, M2=LOW
// EN can be tied LOW (enabled)

#include <Arduino.h>

// Pin setup (only these outputs are used)
#define stepPin 25
#define dirPin  26
#define buttonPin 27

// Motor / mechanics (adapted from your winding_line_control logic)
#define stepsPerRev 200.0      // new motor: 200 full steps/rev
#define microsteps 32.0         // 1/32 microstepping
#define leadScrewPitch 0.002   // m/rev
#define guideMaxPosition 0.05  // m
#define spoolRadius 0.053      // m
#define filamentDiameter 0.00285 // m

const static double stepsPerRevolution = stepsPerRev * microsteps;
const static double ratio = filamentDiameter / leadScrewPitch;

// Constant test input (what you asked for)
const double TEST_FILAMENT_SPEED = 0.5; // m/s

volatile double guidePosition = 0.0; // m
volatile int layerNumber = 0;
volatile int stepDirection = HIGH;   // start moving forward
volatile double stepperSpeed = 0.0;  // steps/s for diagnostics
unsigned long microsPrevStep = 0;
unsigned long lastPrint = 0;

void stepperControl(unsigned long microsCurrent, double filamentSpeed) {
  // Same core math as your original code
  double spoolOmega = filamentSpeed / (spoolRadius + layerNumber * filamentDiameter);
  double guideOmega = spoolOmega * ratio;
  stepperSpeed = guideOmega / (2.0 * PI) * stepsPerRevolution;

  if (stepperSpeed <= 0.0) return;
  double stepIntervalSeconds = 1.0 / stepperSpeed;

  // Count one layer when reaching max while moving forward, then reverse.
  if (guidePosition >= guideMaxPosition && stepDirection == HIGH) {
    guidePosition = guideMaxPosition;
    layerNumber++;
    stepDirection = LOW;
    digitalWrite(dirPin, stepDirection);
  }

  // Count one layer when reaching zero while moving backward, then reverse.
  if (guidePosition <= 0.0 && stepDirection == LOW) {
    guidePosition = 0.0;
    layerNumber++;
    stepDirection = HIGH;
    digitalWrite(dirPin, stepDirection);
  }

  // Same non-blocking toggle stepping style from original
  if (microsCurrent - microsPrevStep >= (unsigned long)(1000000.0 * stepIntervalSeconds * 0.5)) {
    unsigned int state = digitalRead(stepPin);
    digitalWrite(stepPin, !state);
    microsPrevStep = microsCurrent;

    // Update position on rising edge only (same as original)
    //if (state == HIGH) {
      if (stepDirection == HIGH) {
        guidePosition += leadScrewPitch / stepsPerRevolution;
      } else {
        guidePosition -= leadScrewPitch / stepsPerRevolution;
      }
    //}
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, stepDirection);

  Serial.println("Press start button to begin...");

  // Wait for button press (active LOW) and release before starting.
  while (digitalRead(buttonPin) == HIGH) {
    delay(5);
  }
  delay(30);
  while (digitalRead(buttonPin) == LOW) {
    delay(5);
  }

  Serial.println("Standalone stepperControl test started.");
}

void loop() {
  unsigned long microsCurrent = micros();
  stepperControl(microsCurrent, TEST_FILAMENT_SPEED);

  // Diagnostics every 500 ms
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("guidePosition(m): ");
    Serial.print(guidePosition, 5);
    Serial.print(" | dir: ");
    Serial.print(stepDirection);
    Serial.print(" | stepperSpeed(steps/s): ");
    Serial.print(stepperSpeed, 2);
    Serial.print(" | layer: ");
    Serial.println(layerNumber);
  }
}