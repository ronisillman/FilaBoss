//Stepper motor control for the filament guide on a leadscrew

#define stepPin 3
#define dirPin 4
#define enablePin 5
#define limitSwitchPin 2

#define stepSize 20.0 //degrees per step
#define microsteps 16.0 //microsteps per full step
#define stepsPerRevolution 360.0/stepSize*microsteps
#define leadScrewPitch 0.005 //m per revolution

#define stepsPerMM stepsPerRevolution/leadScrewPitch

#define guideMaxPosition 0.05 //m, maximum position of the filament guide
#define spoolRadius 0.053 //m, start radius of the filament spool
#define filamentDiameter 0.00285 //m, diameter of the filament
#define spoolWidth 0.045 //m, width of the filament spool
#define ratio filamentDiameter/leadScrewPitch //gear ratio between spool and guide, set to 1 for direct drive

double guidePosition = 0.0; //m, current position of the filament guide
int layerNumber = 0; //current layer number, used for testing
//Test parameters
double speed = 0.05; //m/s

// Stepper timing variables
int stepsRemaining = 0;
int stepDirection = LOW;
unsigned long lastStepTime = 0;
const unsigned long stepInterval = 1000; // microseconds between steps

// Layer timing variables
unsigned long lastLayerUpdateTime = 0;
const unsigned long layerUpdateInterval = 2000000; // microseconds (2 seconds)
unsigned long lastSerialPrintTime = 0;
const unsigned long serialPrintInterval = 2000000; // microseconds (2 seconds)

void setup() {

    Serial.begin(9600);
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB port only
    }
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(limitSwitchPin, INPUT_PULLUP);

    digitalWrite(dirPin, LOW); // Set direction towards the limit switch
    digitalWrite(enablePin, HIGH); // Enable the stepper driver

    // Home the stepper using clock-based timing
    lastStepTime = micros();
    while (digitalRead(limitSwitchPin) == HIGH) {
        unsigned long currentTime = micros();
        if (currentTime - lastStepTime >= stepInterval) {
            digitalWrite(stepPin, !digitalRead(stepPin)); // Toggle step pin
            lastStepTime = currentTime;
        }
    }
    
    digitalWrite(stepPin, LOW);
    lastLayerUpdateTime = millis();
    lastSerialPrintTime = millis();
}

void loop() {
    unsigned long currentMillis = millis();
    unsigned long currentMicros = micros();
    
    double spoolOmega = speed / (spoolRadius+layerNumber*filamentDiameter); // Calculate angular speed of the spool
    double guideOmega = spoolOmega * ratio; // Calculate the required angular speed of the guide
    double stepperSpeed = guideOmega/(2 * PI) * stepsPerRevolution; // Convert to steps per second
    double stepIntervalSeconds = 1.0 / stepperSpeed; // Calculate interval between steps in seconds
    long stepInterval = stepIntervalSeconds * 1000000; // Convert to microseconds

    // Check if it's time to update the layer number and reverse direction
    if (guidePosition >= guideMaxPosition) {
        layerNumber++;
        stepDirection = !stepDirection; // Reverse direction for next layer
        digitalWrite(dirPin, stepDirection); // Set direction
    }
    else if (digitalRead(limitSwitchPin) == LOW) { // If limit switch is triggered, reverse direction
        layerNumber++;
        stepDirection = !stepDirection; // Reverse direction for next layer
        digitalWrite(dirPin, stepDirection); // Set direction
        guidePosition = 0.0; // Reset guide position to 0 when limit switch is triggered    
    }

    // Handle stepper motor movement (non-blocking)
    
    if (currentMicros - lastStepTime >= stepInterval/2) { // Toggle step pin at half the interval for proper timing
        digitalWrite(stepPin, !digitalRead(stepPin)); // Toggle step pin
        lastStepTime = currentMicros;
        if (digitalRead(stepPin) == HIGH) { // Only count steps on the rising edge
            if (stepDirection == HIGH) {
                guidePosition += leadScrewPitch / stepsPerRevolution; // Update guide position
            } else {
                guidePosition -= leadScrewPitch / stepsPerRevolution; // Update guide position
            }
    }
    

 /*   // Print serial data (non-blocking)
    if (currentMillis - lastSerialPrintTime >= serialPrintInterval) {
        lastSerialPrintTime = currentMillis;
        
        Serial.print("Layer: ");
        Serial.print(layerNumber);
        Serial.print(" | Guide Position: ");
        Serial.print(guidePosition);
        Serial.print(" mm | Steps remaining: ");
        Serial.println(stepsRemaining);
    }
    */
}