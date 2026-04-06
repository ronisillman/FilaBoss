#define RXD2 25
#define TXD2 26

void setup() {
  Serial.begin(115200);          // USB monitor
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART from Pi
}

void loop() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    Serial.println("Received from Pi:");
    Serial.println(data);
  }
}