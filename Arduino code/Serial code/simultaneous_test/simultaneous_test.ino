int counter = 1;

void setup() {
  Serial.begin(115200); // USB or GPIO UART
}

void loop() {
  // Send message
  Serial.print("Hello ESP ");
  Serial.println(counter);

  // Read incoming messages and print them
  while (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.length() > 0) {
      Serial.println("Received: " + msg);
    }
  }

  counter++;
  delay(1000);
}