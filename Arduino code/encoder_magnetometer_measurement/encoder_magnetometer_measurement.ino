#include <Wire.h>
#include <Adafruit_AS5600.h>

// --- Values from your Winding_line_control.ino ---
#define encoderPinA 2
#define encoderPinB 4
#define rollerRadius 0.0119
#define encoderResolution 600.0

Adafruit_AS5600 as5600;

// Encoder state
volatile long encoderTicks = 0;
volatile uint8_t lastAState = 0;

// Timing/state for speed calc
unsigned long lastSampleMs = 0;
long prevEncoderTicks = 0;
uint16_t prevAsAngle = 0;

void encoderISR() {
  uint8_t a = digitalRead(encoderPinA);
  uint8_t b = digitalRead(encoderPinB);

  if (a != lastAState) {
    encoderTicks += (b != a) ? 1 : -1;   // quadrature direction
    lastAState = a;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  lastAState = digitalRead(encoderPinA);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);

  Wire.begin();
  if (!as5600.begin()) {
    Serial.println("AS5600 not found!");
    while (1) delay(100);
  }

  prevAsAngle = as5600.getAngle(); // 0..4095
  lastSampleMs = millis();

  Serial.println("enc_speed_mm_s,as5600_speed_mm_s,mixed_speed_mm_s");
}

void loop() {
  const unsigned long sampleMs = 50; // 20 Hz output
  unsigned long now = millis();
  if (now - lastSampleMs < sampleMs) return;

  float dt = (now - lastSampleMs) / 1000.0f;
  lastSampleMs = now;

  // ----- Encoder speed -----
  noInterrupts();
  long ticks = encoderTicks;
  interrupts();

  long deltaTicks = ticks - prevEncoderTicks;
  prevEncoderTicks = ticks;

  float encRevPerSec = ((float)deltaTicks / encoderResolution) / dt;
  float encSpeed = fabs(encRevPerSec * (2.0f * PI * rollerRadius)); // m/s

  // ----- AS5600 speed -----
  uint16_t angleNow = as5600.getAngle(); // 0..4095
  int16_t d = (int16_t)angleNow - (int16_t)prevAsAngle;

  // unwrap around 0/4095 boundary
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;

  prevAsAngle = angleNow;

  float asRevPerSec = ((float)d / 4096.0f) / dt;
  float asSpeed = fabs(asRevPerSec * (2.0f * PI * rollerRadius)); // m/s


  // Print in mm/s
  Serial.print("Encoder speed: ");
  Serial.print(encSpeed * 1000.0f, 3);
  Serial.print("||");
  Serial.print("Magnetometer speed: ");
  Serial.println(asSpeed * 1000.0f, 3);
}