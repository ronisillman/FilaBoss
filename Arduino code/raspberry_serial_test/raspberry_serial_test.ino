#include <Arduino.h>

// ESP32 UART2 pins (adjust only if your wiring uses different GPIOs)
static const int PI_RX_PIN = 25; // ESP32 RX <- Raspberry Pi TX
static const int PI_TX_PIN = 26; // ESP32 TX -> Raspberry Pi RX
static const uint32_t UART_BAUD = 115200;

HardwareSerial PiSerial(2);

String rxLine;
unsigned long lastHeartbeatMs = 0;
unsigned long counter = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  PiSerial.begin(UART_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  Serial.println("ESP32 simple UART test started");
  PiSerial.println("READY");
}

void handleIncomingFromPi() {
  while (PiSerial.available() > 0) {
    char c = (char)PiSerial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rxLine.length() > 0) {
        Serial.print("RX from Pi: ");
        Serial.println(rxLine);

        PiSerial.print("ECHO:");
        PiSerial.println(rxLine);
      }
      rxLine = "";
    } else {
      rxLine += c;
      if (rxLine.length() > 120) {
        rxLine = "";
      }
    }
  }
}

void sendHeartbeatToPi() {
  unsigned long now = millis();
  if (now - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = now;
    PiSerial.print("HB ");
    PiSerial.print(now);
    PiSerial.print(" #");
    PiSerial.println(counter++);
  }
}

void loop() {
  handleIncomingFromPi();
  sendHeartbeatToPi();
}