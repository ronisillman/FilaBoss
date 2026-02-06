#define encoderPin 2

volatile long pulseCount = 0;
volatile bool lastState = LOW;

const int motorRollerPin = 3;
const int motorSpoolPin = 11;

int speed_Roller = 20;
int speed_Spool = 15;

void setup() {
  pinMode(encoderPin, INPUT);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, CHANGE);
  analogWrite(motorRollerPin, speed_Roller);
  analogWrite(motorSpoolPin, speed_Spool);
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 1000) {
    noInterrupts();
    long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    Serial.print("Pulses in last 1 s: ");
    Serial.println(pulses);
    lastTime = millis();
  }
}

void countPulse() {
  static unsigned long lastBounceTime = 0;
  unsigned long now = micros();  // micros() for better debounce resolution
  
  // Debounce with 1000µs (1ms) threshold
  if (now - lastBounceTime > 1000) {
    bool state = digitalRead(encoderPin);
    if (state == HIGH && lastState == LOW) {
      pulseCount++;
    }
    lastState = state;
    lastBounceTime = now;
  }
}