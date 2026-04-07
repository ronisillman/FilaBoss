int counter = 1;

void setup() {
    Serial.begin(115200); 
    Serial2.begin(115200, SERIAL_8N1, 25, 26);
}

void loop() {
  // Read incoming messages and print them
  while (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      Serial.println("Received: " + msg);
    }
  }

  Serial2.print("Hello Pi ");
  Serial2.println(counter);
  counter++;
  delay(1000);
}