#include <Arduino.h>

static const uint8_t UART1_RX = 16;
static const uint8_t UART1_TX = 17;
static const uint32_t UART_BAUD = 115200;

HardwareSerial U1(1);

uint32_t sentCount = 0;
uint32_t recvCount = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("UART1 loopback test");
  Serial.println("Jumper GPIO17 (TX1) to GPIO16 (RX1) directly.");

  U1.begin(UART_BAUD, SERIAL_8N1, UART1_RX, UART1_TX);
  delay(50);
}

void loop() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs >= 500) {
    lastMs = millis();

    const uint8_t b = (uint8_t)(sentCount & 0xFF);
    U1.write(b);
    sentCount++;
  }

  while (U1.available() > 0) {
    const int v = U1.read();
    recvCount++;
    Serial.print("RX byte: ");
    Serial.print(v);
    Serial.print(" | sent=");
    Serial.print(sentCount);
    Serial.print(" recv=");
    Serial.println(recvCount);
  }

  static uint32_t reportMs = 0;
  if (millis() - reportMs >= 2000) {
    reportMs = millis();
    Serial.print("summary sent=");
    Serial.print(sentCount);
    Serial.print(" recv=");
    Serial.println(recvCount);
  }
}
