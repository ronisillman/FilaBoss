const int motorRollerPin = 3;
const int motorSpoolPin = 11;

int speed_Roller = 30;
int speed_Spool = 20;

void setup() {
  pinMode(motorRollerPin, OUTPUT);
  pinMode(motorSpoolPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Beginning motor code.");

  // Start motors immediately
  analogWrite(motorRollerPin, speed_Roller);
  analogWrite(motorSpoolPin, speed_Spool);
}

void loop() {
  Serial.print("Roller Speed: ");
  Serial.print(speed_Roller);
  Serial.print(" | Spool Speed: ");
  Serial.println(speed_Spool);

  analogWrite(motorRollerPin, speed_Roller);
  analogWrite(motorSpoolPin, speed_Spool);
}
