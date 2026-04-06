#include <Arduino.h>

// ESP32 UART2 pins
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
  Serial.println("ESP32 UART test started");
  PiSerial.println("ESP32_READY");
}

void handleIncomingFromPi() {
  while (PiSerial.available() > 0) {
    char c = (char)PiSerial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rxLine.length() > 0) {
        Serial.print("From Pi: ");
        Serial.println(rxLine);

        if (rxLine.startsWith("CMD:")) {
          String cmd = rxLine.substring(4);
          PiSerial.print("ACK:");
          PiSerial.println(cmd);

          if (cmd == "PING") {
            PiSerial.println("EVT:PONG");
          } else if (cmd == "LED_ON") {
            PiSerial.println("EVT:LED_STATE:ON");
          } else if (cmd == "LED_OFF") {
            PiSerial.println("EVT:LED_STATE:OFF");
          } else {
            PiSerial.println("EVT:UNKNOWN_CMD");
          }
        } else if (rxLine.startsWith("ACK:")) {
          // Ignore ACK frames to prevent ACK feedback loops.
        } else {
          PiSerial.println("EVT:EXPECTED_CMD_PREFIX");
        }
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
    PiSerial.print("EVT:ESP_HEARTBEAT,ms=");
    PiSerial.print(now);
    PiSerial.print(",count=");
    PiSerial.println(counter++);
  }
}

void loop() {
  handleIncomingFromPi();
  sendHeartbeatToPi();
}