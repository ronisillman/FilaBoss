//Stepper motor control for the filament guide on a leadscrew

#define stepPin 3
#define dirPin 4
#define enablePin 5
#define limitSwitchPin 2

#define stepSize 20
#define microsteps 16
#define stepsPerRevolution 360/stepSize*microsteps
#define stepperPitch 5 //mm per revolution

//Test parameters
double speed = 0.05; //m/s

void setup() {

    Serial.begin(9600);
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB port only
    }
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(limitSwitchPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(limitSwitchPin), limitSwitchTriggered, FALLING);
    

    while (digitalRead(limitSwitchPin) == HIGH) {
        // Move the stepper motor towards the limit switch until it is triggered
        digitalWrite(dirPin, LOW); // Set direction towards the limit switch
        digitalWrite(enablePin, LOW); // Enable the stepper driver
        for (int i = 0; i < stepsPerRevolution; i++) {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(1000); // Adjust delay for desired speed
            digitalWrite(stepPin, LOW);
            delayMicroseconds(1000);
        }
    }
}
void limitSwitchTriggered() {
    // Stop the stepper motor when the limit switch is triggered
    digitalWrite(enablePin, HIGH); // Disable the stepper driver
    Serial.println("Limit switch triggered! Stepper motor stopped.");
}
void loop() {
  // put your main code here, to run repeatedly:

}