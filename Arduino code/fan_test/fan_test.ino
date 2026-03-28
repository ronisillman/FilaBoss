#include <Arduino.h>
#include "driver/pcnt.h"

// ---------------------------
// Pin definitions
// ---------------------------
const int fanPwmPin  = 33; // PWM output
const int fanTachPin = 32; // Tach input (open-collector)
const int pwmFreq    = 25000; // 25 kHz PWM
const int pwmResolution = 8;   // 8-bit PWM
const int pwmMax     = 255;    // max duty
const int tachPulsesPerRev = 2; // 2 pulses per revolution

// ---------------------------
// PWM variables
// ---------------------------
uint8_t dutyPercent = 40;

// ---------------------------
// PCNT configuration
// ---------------------------
pcnt_unit_t pcntUnit = PCNT_UNIT_0;
pcnt_config_t pcntConfig;

// Count interval
unsigned long lastPrintMs = 0;
const unsigned long printInterval = 1000; // ms

// ---------------------------
// Function to set fan PWM
// ---------------------------
void setFanDutyPercent(uint8_t percent) {
  if (percent > 100) percent = 100;
  dutyPercent = percent;
  int dutyRaw = map(dutyPercent, 0, 100, 0, pwmMax);
  ledcWrite(fanPwmPin, dutyRaw); // write PWM
}

// ---------------------------
// Setup
// ---------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  // PWM setup
  ledcAttach(fanPwmPin, pwmFreq, pwmResolution);
  setFanDutyPercent(dutyPercent);

  // ---------------------------
  // PCNT setup for tach
  // ---------------------------
  pcntConfig.pulse_gpio_num = fanTachPin;
  pcntConfig.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcntConfig.channel = PCNT_CHANNEL_0;
  pcntConfig.unit = pcntUnit;
  pcntConfig.pos_mode = PCNT_COUNT_INC; // count rising edges
  pcntConfig.neg_mode = PCNT_COUNT_DIS; // ignore falling edges
  pcntConfig.lctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.counter_h_lim = 32767;
  pcntConfig.counter_l_lim = 0;
  pcnt_unit_config(&pcntConfig);

  // Filter to ignore glitches < 5 us
  pcnt_set_filter_value(pcntUnit, 5); // 5 us
  pcnt_filter_enable(pcntUnit);

  // Clear and start counting
  pcnt_counter_clear(pcntUnit);
  pcnt_counter_resume(pcntUnit);

  Serial.println("ESP32 Fan Test with PCNT Started");
  Serial.println("Send 0..100 in Serial Monitor to set PWM duty %");
  Serial.println("Example: 30");
}

// ---------------------------
// Loop
// ---------------------------
void loop() {
  // Serial input for PWM duty
  if (Serial.available()) {
    int val = Serial.parseInt();
    if (val >= 0 && val <= 100) {
      setFanDutyPercent(val);
      Serial.print("Set duty to ");
      Serial.print(dutyPercent);
      Serial.println("%");
    }
    while (Serial.available()) Serial.read(); // clear buffer
  }

  // Print RPM every second
  unsigned long now = millis();
  if (now - lastPrintMs >= printInterval) {
    int16_t pulseCount = 0;
    pcnt_get_counter_value(pcntUnit, &pulseCount);
    pcnt_counter_clear(pcntUnit);

    // Convert pulses to RPM
    // pulses per second * 60 / pulses per rev
    float rpm = (pulseCount * 60.0f) / tachPulsesPerRev;

    Serial.print("Duty: ");
    Serial.print(dutyPercent);
    Serial.print("%, Tach pulses/s: ");
    Serial.print(pulseCount);
    Serial.print(", RPM: ");
    Serial.println((int)rpm);

    lastPrintMs = now;
  }
}