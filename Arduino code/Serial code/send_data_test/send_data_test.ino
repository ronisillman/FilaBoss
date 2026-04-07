void setup() {
  Serial2.begin(115200, SERIAL_8N1, 25, 26);
}

void loop() {
  Serial2.println("Hello Pi");
  delay(1000);
}