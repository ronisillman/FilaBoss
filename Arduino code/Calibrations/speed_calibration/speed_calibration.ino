// Standalone ESP32 sketch:
// - PCNT-based encoder pulse source
// - SAME speed/decay functions as your code
// - Traveled distance from integrated speed
// - Pulley motor control with CONSTANT output (no PID)

#include <Arduino.h>
#include "driver/pcnt.h"

// ---------------- Pins ----------------
#define encoderPinA 4     // PCNT-capable
#define encoderPinB 5     // PCNT-capable
#define motorPulleyPin 18 // pulley/roller motor PWM output

// ---------------- Constants (same speed math) ----------------
#define rollerRadius 0.0119
#define encoderEdgesPerPulse 2.0
#define encoderResolution 600.0
#define SpeedFilterAlpha 0.03

#define PCNT_UNIT_USED PCNT_UNIT_0
#define PCNT_H_LIM 32767

// Constant motor command (0..255 for ESP32 analogWrite-style API in Arduino core)
const int pulleyPwm = 120;

// ---------------- State ----------------
double speed = 0.0;               // m/s
double traveledDistance = 0.0;    // m

unsigned long lastEncoderPeriod = 0;
unsigned long encoderPrevTime = 0;
unsigned long lastDiagnoseTime = 0;

volatile bool encoderPulseEvent = false;
int16_t prevPcntCount = 0;
bool hasValidPulse = false;

unsigned long lastMeasurementMs = 0;
unsigned long lastDistanceUpdateMs = 0;

const double speedCal = 500.0 / 703.23f; 

// ---------------- Timing ----------------
const unsigned long MEASUREMENT_PERIOD_MS = 5;
const unsigned long DIAGNOSE_PERIOD_MS = 500;
const unsigned long STOP_TIMEOUT_MS = 300;
const float SPEED_DEADBAND_MPS = 0.0002f; // 0.2 mm/s

// ---------------- PCNT ----------------
void initPCNT() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = encoderPinA,
        .ctrl_gpio_num = encoderPinB,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = -PCNT_H_LIM,
        .unit = PCNT_UNIT_USED,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_USED, 100);
    pcnt_filter_enable(PCNT_UNIT_USED);
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_counter_resume(PCNT_UNIT_USED);
}

void pollPcntPulseEvent() {
    int16_t pcntCount = 0;
    pcnt_get_counter_value(PCNT_UNIT_USED, &pcntCount);

    int16_t countDelta = pcntCount - prevPcntCount;
    if (countDelta != 0) {
        encoderPulseEvent = true;
        prevPcntCount = pcntCount;
    }
}

// ---------------- SAME speed functions ----------------
void updateSpeedByPulse() {
    unsigned long currentTime = millis();
    unsigned long period = currentTime - encoderPrevTime; // Time since last pulse in milliseconds
    if (period == 0) return; // Avoid division by zero, should not happen with proper timing
    // With CHANGE interrupt on channel A, we get 2 edges per encoder pulse.
    float revPerSec = 1000.0f / ((float)period * encoderResolution * encoderEdgesPerPulse); // revolutions per second
    float rawSpeed = revPerSec * (2.0f * PI * rollerRadius); // Calculate speed in m/s based on roller radius
    rawSpeed *= speedCal;
    speed += SpeedFilterAlpha * (rawSpeed - speed); // Apply low-pass filter to smooth speed measurement
    lastEncoderPeriod = period;
    encoderPrevTime = currentTime;
    hasValidPulse = true;
}

void decaySpeed() {
    unsigned long currentTime = millis();
    unsigned long gap_period = currentTime - encoderPrevTime; // Time since last pulse in milliseconds

    if (!hasValidPulse) {
        speed = 0.0;
        return;
    }

    if (gap_period > STOP_TIMEOUT_MS) {
        speed = 0.0;
        return;
    }

    if (gap_period > lastEncoderPeriod){ // Wait for a full period to elapse before decaying speed, ensures we only decay after missing a pulse
    float revPerSec = 1000.0 / ((float)gap_period * encoderResolution * encoderEdgesPerPulse); // Match encoder edge counting in ISR
    float rawSpeed = revPerSec * (2.0 * PI * rollerRadius); // Calculate raw speed in m/s based on roller radius
    rawSpeed *= speedCal;
    speed += SpeedFilterAlpha * (rawSpeed - speed); // Apply low-pass filter to smooth speed measurement
    }

    if (fabs(speed) < SPEED_DEADBAND_MPS) {
        speed = 0.0;
    }
}

// ---------------- Distance ----------------
void updateDistance() {
    unsigned long nowMs = millis();
    unsigned long dtMs = nowMs - lastDistanceUpdateMs;
    if (dtMs == 0) return;

    double dt = (double)dtMs / 1000.0;
    if (fabs(speed) >= SPEED_DEADBAND_MPS) {
        traveledDistance += speed * dt;
    }
    lastDistanceUpdateMs = nowMs;
}

// ---------------- Constant pulley control ----------------
void motorPulleyControlConstant() {
    analogWrite(motorPulleyPin, pulleyPwm);
}

// ---------------- Diagnostics ----------------
void diagnose(unsigned long interval) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastDiagnoseTime >= interval) {
        lastDiagnoseTime = currentMillis;

        Serial.print("pulley_pwm:");
        Serial.print(pulleyPwm);
        Serial.print(",measured_speed_mm_s:");
        Serial.print(speed * 1000.0f);
        Serial.print(",distance_m:");
        Serial.print(traveledDistance, 6);
        Serial.print(",distance_mm:");
        Serial.println(traveledDistance * 1000.0f, 2);
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);

    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(motorPulleyPin, OUTPUT);

    initPCNT();
    pcnt_get_counter_value(PCNT_UNIT_USED, &prevPcntCount);

    encoderPrevTime = millis();
    lastEncoderPeriod = 100;
    lastMeasurementMs = millis();
    lastDistanceUpdateMs = millis();

    // Start pulley motor at constant command
    motorPulleyControlConstant();

    Serial.println("PCNT speed + distance + constant pulley control started.");
}

void loop() {
    pollPcntPulseEvent();

    if (encoderPulseEvent) {
        noInterrupts();
        encoderPulseEvent = false;
        interrupts();
        updateSpeedByPulse();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastMeasurementMs >= MEASUREMENT_PERIOD_MS) {
        decaySpeed();
        lastMeasurementMs = currentMillis;
    }

    // Keep constant motor command active
    motorPulleyControlConstant();

    updateDistance();
    diagnose(DIAGNOSE_PERIOD_MS);
}