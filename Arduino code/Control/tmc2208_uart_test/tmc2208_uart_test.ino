#include <Arduino.h>
#include <TMCStepper.h>

// IMPORTANT:
// On classic ESP32 (WROOM/WROVER), GPIO9/GPIO10 are used by SPI flash and must not be used.
// Use UART1 defaults RX=16, TX=17 for those boards.
// If you are on ESP32-S3/C3 and GPIO9/10 are actually free on your module,
// replace these with 9 and 10.
static const uint8_t TMC_RX1_PIN = 12;
static const uint8_t TMC_TX1_PIN = 13;

// Optional driver enable pin. Set to -1 if EN is hard-wired.
static const int8_t TMC_EN_PIN = -1;

// TMC2208 default UART baud rate is typically 115200.
static const uint32_t TMC_BAUD = 115200;

// Set this to your driver's sense resistor value (commonly 0.11 ohm).
static constexpr float R_SENSE = 0.11f;

HardwareSerial TMCSerial(1);
TMC2208Stepper driver(&TMCSerial, R_SENSE);

static uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("TMC2208 UART test starting...");
  Serial.print("Using UART1 pins: RX=");
  Serial.print(TMC_RX1_PIN);
  Serial.print(", TX=");
  Serial.println(TMC_TX1_PIN);

  if (TMC_EN_PIN >= 0) {
    pinMode(TMC_EN_PIN, OUTPUT);
    digitalWrite(TMC_EN_PIN, LOW);  // Most modules use active-low EN
  }

  TMCSerial.begin(TMC_BAUD, SERIAL_8N1, TMC_RX1_PIN, TMC_TX1_PIN);
  delay(50);

  driver.begin();
  driver.pdn_disable(true);      // Force UART control
  driver.mstep_reg_select(true); // Microstep config from UART
  driver.I_scale_analog(false);  // Use internal current reference
  driver.toff(5);                // Enable driver stage
  driver.blank_time(24);
  driver.rms_current(500);       // 500 mA test current
  driver.microsteps(16);
  driver.pwm_autoscale(true);

  const uint8_t conn = driver.test_connection();
  Serial.print("test_connection() = ");
  Serial.println(conn);
  Serial.println("Expected: 0 means UART responds.");
  Serial.println("Now printing IFCNT/IOIN every second...");
}

void loop() {
  if (millis() - lastPrintMs < 1000) {
    return;
  }
  lastPrintMs = millis();

  const uint8_t ifcntBefore = driver.IFCNT();

  // Write something so IFCNT should increment when UART works.
  static bool toggle = false;
  driver.rms_current(toggle ? 700 : 300);
  toggle = !toggle;

  delay(5);
  const uint8_t ifcntAfter = driver.IFCNT();

  const uint32_t ioin = driver.IOIN();
  const uint32_t drvStatus = driver.DRV_STATUS();

  Serial.print("IFCNT before/after: ");
  Serial.print(ifcntBefore);
  Serial.print(" -> ");
  Serial.print(ifcntAfter);
  Serial.print(" | IOIN=0x");
  Serial.print(ioin, HEX);
  Serial.print(" | DRV_STATUS=0x");
  Serial.println(drvStatus, HEX);

  if (ifcntAfter == ifcntBefore) {
    Serial.println("WARN: IFCNT did not change. Check UART wiring, common GND, and PDN_UART link.");
  } else {
    Serial.println("OK: UART write acknowledged by TMC2208.");
  }
}
