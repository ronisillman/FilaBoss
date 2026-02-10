//Stepper motor control for the filament guide on a leadscrew
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "pid.h"

//two current sensors
Adafruit_INA219 ina219_spool(0x40);
Adafruit_INA219 ina219_roller(0x41);
PID SpoolPID(0.05, 0.15, 0.0); // Initialize PID controller with example gains
PID RollerPID(1000, 0.3, 0.0); // Initialize PID controller with example gains
 
//pin setup
#define motorRollerPin 3
#define motorSpoolPin 11
#define stepPin 9
#define dirPin 8
#define enablePin 5
#define limitSwitchPin 6
#define encoderPinA 2 // CLK pin
#define encoderPinB 4 // DT pin

#define stepSize 20.0 //degrees per step
#define microsteps 16.0 //microsteps per full step
#define stepsPerRevolution 360.0/stepSize*microsteps
#define leadScrewPitch 0.005 //m per revolution
#define ApproachStepInterval 1000.0 // microseconds between steps

#define stepsPerMM stepsPerRevolution/leadScrewPitch

#define guideMaxPosition 0.05 //m, maximum position of the filament guide
#define spoolRadius 0.053 //m, start radius of the filament spool
#define filamentDiameter 0.00285 //m, diameter of the filament
#define spoolWidth 0.045 //m, width of the filament spool
#define ratio filamentDiameter/leadScrewPitch //gear ratio between spool and guide, set to 1 for direct drive
#define rollerRadius 0.012 //m, radius of the roller in contact with the filament, used for speed calculation

#define encoderResolution 30.0 //pulses per revolution for the encoder


// Filter variables for measurment smoothing
#define CurrentfilterAlpha 0.1 // Smoothing factor for current measurements
#define SpeedFilterAlpha 0.1 // Smoothing factor for speed measurements

double guidePosition = 0.0; //m, current position of the filament guide
int layerNumber = 0; //current layer number, used for testing

//Test parameters   
float targetSpeed = 0.02; //m/s, desired filament speed
double SetTorqueCurrent = 50; //mA, example torque setpoint for testing

// Calibration variables
float noLoadCurrent_Spool = 0.0; //mA, base no-load current for spool motor
float noLoadCurrent_Roller = 0.0; //mA, base no-load current for roller motor
double currentMeasurement_Spool = 0.0; //mA, current measurement for spool motor
double currentMeasurement_Roller = 0.0; //mA, current measurement for roller motor

// Stepper timing variables
int stepDirection = LOW;
unsigned long microsPrevStep = 0;

// Encoder variables for speed measurement
int encoderCount = 0;
int encoderPinA_prev;
int encoderPinA_value;
unsigned long encoderPrevTime = 0;
bool bool_CW;
double speed = 0.0; //m/s, current speed of the filament, calculated from encoder counts

unsigned long lastDiagnoseTime = 0;
unsigned long lastEncoderPeriod = 0;

void setup() {

    Serial.begin(9600);
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB port only
    }

    pinSetup();
    initI2CPeripherals();
    
    // Set output limits for PID controllers
    SpoolPID.setOutputLimits(0, 127); // Set output limits for spool motor control (0-255 for PWM)
    RollerPID.setOutputLimits(0, 127); // Set output limits for roller motor control (0-255 for PWM)

    // Run calibration 
    //HomingAndCalibration(5000, 100); // Run calibration for 5 seconds per motor, sampling every 100 ms
    encoderPinA_prev = digitalRead(encoderPinA);
    
}

void loop() {
    unsigned long currentMillis = millis();
    unsigned long microsCurrent = micros();
    countPulse(); // Update encoder count and calculate actual speed'

    //stepperControl(microsCurrent, speed);

    // Placeholder for actual speed measurement from encoder or other sensor, using current as a proxy for testing
    //float measureSpeed = (ina219_roller.getCurrent_mA()/noLoadCurrent_Roller);

    motorControl(targetSpeed, speed);
    diagnose(1000); // Print diagnostics every 1000 ms (1 second)
}

/// @brief Print diagnostic information to the serial monitor at a specified interval
/// @param interval The interval in milliseconds between diagnostic prints
void diagnose(unsigned long interval) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastDiagnoseTime >= interval) { // Print diagnostics every 1 second
        lastDiagnoseTime = currentMillis;
        Serial.print("Filament Speed(m/s): ");
        Serial.print(speed*1000.0);
        Serial.print(" | Set Speed(m/s): ");
        Serial.print(targetSpeed);
        Serial.print(" | Spool Current(mA): ");
        Serial.print(currentMeasurement_Spool);
        Serial.print(" | Spool Control Signal: ");
        Serial.print(SpoolPID.getOutput());
        Serial.print(" | Roller Control Signal: ");
        Serial.print(RollerPID.getOutput());
        Serial.print(" | Guide Position(m): ");
        Serial.print(guidePosition);
        Serial.print(" | Layer Number: ");
        Serial.print(layerNumber);
        // Add more diagnostic information as needed
        Serial.println();
    }
}

/// @brief 
/// @param setSpeed Desired speed for the roller motor in m/s, used to calculate the speed error for the PID controller.
/// @param actualSpeed Actual speed of the roller motor in m/s, used to calculate the speed error for the PID controller.
/// TODO: Implement cascaded control to prevent spool and roller speed missmatch
void motorControl(float setSpeed, float actualSpeed) {
    // Calculate error from no-load current
    float torqueCurrent = currentMeasurement_Spool - noLoadCurrent_Spool*SpoolPID.getOutput()/255.0; // Subtract scaled no-load current from actual current to get torque-related current for spool
    SpoolPID.setSetpoint(SetTorqueCurrent);
    //SpoolPID.setSetpoint(0.2);
    RollerPID.setSetpoint(setSpeed); // Set speed setpoint for roller PID (converted to mm/s for better resolution)
    Serial.print(setSpeed);
    Serial.print(" |");
    Serial.println(setSpeed);
    // Update PID controllers
    int controlSignal_Spool = SpoolPID.compute(currentMeasurement_Spool);
    int controlSignal_Roller = RollerPID.compute(actualSpeed); // Compute control signal for roller PID (converted to mm/s for better resolution)
    // Apply control signals to motors (using PWM for speed control)
    //analogWrite(motorSpoolPin, controlSignal_Spool);
    int spd = 30; // Placeholder for testing, replace with controlSignal_Spool for actual control
    analogWrite(motorRollerPin, spd);
    analogWrite(motorSpoolPin, spd*(rollerRadius/spoolRadius)+12);
    //analogWrite(motorRollerPin, controlSignal_Roller);
}

/// @brief 
/// @param microsCurrent //current time in microseconds for timing control of the stepper motor
/// @param filamentSpeed //desired filament speed in m/s, used to calculate the required stepper speed for the guide
void stepperControl(unsigned long microsCurrent, double filamentSpeed) {
    double spoolOmega = filamentSpeed / (spoolRadius+layerNumber*filamentDiameter); // Calculate angular speed of the spool
    double guideOmega = spoolOmega * ratio;                                 // Calculate the required angular speed of the guide
    double stepperSpeed = guideOmega/(2.0 * PI) * stepsPerRevolution;       // Convert to steps per second
    double stepIntervalSeconds = 1.0 / stepperSpeed;                        // Calculate interval between steps in seconds

    // Check if it's time to update the layer number and reverse direction
    if (digitalRead(limitSwitchPin) == LOW) {                               // If limit switch is triggered, reverse direction
        layerNumber++;
        stepDirection = !stepDirection;                                     // Reverse direction for next layer
        digitalWrite(dirPin, stepDirection);                                // Set direction
        guidePosition = 0.0;                                                // Reset guide position to 0 when limit switch is triggered    
    }
    else if (guidePosition >= guideMaxPosition) {
        layerNumber++;
        stepDirection = !stepDirection; // Reverse direction for next layer
        digitalWrite(dirPin, stepDirection); // Set direction
    }

    // Handle stepper motor movement (non-blocking)
    if (microsCurrent - microsPrevStep >= stepIntervalSeconds/2) { // Toggle step pin at half the interval for proper timing
        digitalWrite(stepPin, !digitalRead(stepPin)); // Toggle step pin
        microsPrevStep = microsCurrent;
        if (digitalRead(stepPin) == HIGH) { // Only count steps on the rising edge
            if (stepDirection == HIGH) {
                guidePosition += leadScrewPitch / stepsPerRevolution; // Update guide position
            } else {
                guidePosition -= leadScrewPitch / stepsPerRevolution; // Update guide position
            }
        }
    }
}

/// @brief Initialize I2C peripherals (current sensors) and check for their presence. If a sensor is not found, print an error message and halt execution.
void initI2CPeripherals() {
    Wire.begin();
  // Initialize INA219 at 0x40
  if (!ina219_spool.begin()) {
    Serial.println("Failed to find INA219 at address 0x40");
    while (1) {
      delay(10);
    }
  }

  // Initialize INA219 at 0x41
  if (!ina219_roller.begin()) {
    Serial.println("Failed to find INA219 at address 0x41");
    while (1) {
      delay(10);
    }
  }
}

/// @brief Set pin modes for stepper control, limit switch, and motor control pins.
void pinSetup() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(limitSwitchPin, INPUT_PULLUP);
    pinMode(motorRollerPin, OUTPUT);
    pinMode(motorSpoolPin, OUTPUT);
    pinMode (encoderPinA, INPUT);
    pinMode (encoderPinB, INPUT);
}

/// @brief Perform homing and no-load current calibration for the filament guide system.
/// @param calibrationTime_ms Duration in milliseconds for which to run the no-load current calibration. 
/// @param sampleInterval_ms Interval in milliseconds between current samples during calibration.
void HomingAndCalibration(int calibrationTime_ms, int sampleInterval_ms) {
    Serial.println("Starting no-load current calibration and guide homing...");    
    // Non-blocking calibration state and functions
    bool calibrated = false;
    bool homed = true;

    unsigned long calibStartTime = millis();
    unsigned long millisPrev = 0;

    float calib_spool_sum = 0.0;
    int calib_spool_samples = 0;
    float calib_roller_sum = 0.0;
    int calib_roller_samples = 0;

    stepDirection = LOW; // Set initial direction towards the limit switch
    digitalWrite(dirPin, stepDirection); // Set direction towards the limit switch
    digitalWrite(enablePin, HIGH); // Enable the stepper driver
    
    analogWrite(motorRollerPin, 255); // Run roller at full speed for calibration
    analogWrite(motorSpoolPin, 255); // Run spool at full speed for calibration

    while (!calibrated || !homed) {
        unsigned long millisCurrent = millis();
        unsigned long microsCurrent = micros();

        if (!calibrated){
            // Sample current at regular intervals during calibration period
            if (millisCurrent - calibStartTime >= (unsigned long)calibrationTime_ms) {
                calibrated = true;
                noLoadCurrent_Spool = calib_spool_sum / calib_spool_samples;
                noLoadCurrent_Roller = calib_roller_sum / calib_roller_samples;
                Serial.print("Spool no-load current: ");
                Serial.print(noLoadCurrent_Spool);
                Serial.println(" mA");
                Serial.print("Roller no-load current: ");   
                Serial.print(noLoadCurrent_Roller);
                Serial.println(" mA");
            }
            // Accumulate current samples for calibration
            if (millisCurrent - millisPrev >= (unsigned long)sampleInterval_ms) {
                calib_spool_sum += ina219_spool.getCurrent_mA();
                calib_spool_samples++;
                calib_roller_sum += ina219_roller.getCurrent_mA();
                calib_roller_samples++;
                millisPrev = millisCurrent;
            }
        }
        // Handle homing towards the limit switch
        if (!homed) {
            if (digitalRead(limitSwitchPin) == HIGH) { // If limit switch is not triggered, continue homing
                if (microsCurrent - microsPrevStep >= ApproachStepInterval) {
                    digitalWrite(stepPin, !digitalRead(stepPin)); // Toggle step pin
                }
            }
            else {
                homed = true; // Limit switch triggered, homing complete
                Serial.println("Homing complete.");
                stepDirection = !stepDirection; 
                digitalWrite(stepPin, stepDirection); // Set direction for normal operation
            }
        }
        microsPrevStep = microsCurrent;
    }
}

/// @brief Interrupt service routine to count encoder pulses and calculate speed.
void countPulse() {
    unsigned long currentTime = micros();
    unsigned long period = currentTime - encoderPrevTime; // Time since last pulse in microseconds
    encoderPinA_value = digitalRead(encoderPinA);
    if (encoderPinA_value != encoderPinA_prev) { // check if knob is rotating
        // if pin A state changed before pin B, rotation is clockwise
        if (digitalRead(encoderPinB) != encoderPinA_value) {
            encoderCount ++;
            bool_CW = true;
            lastEncoderPeriod = period;
            speed -= (speed - 2.0*PI*rollerRadius*1000000.0/((float)encoderResolution*(float)period))*SpeedFilterAlpha; // Calculate speed in m/s based on encoder counts and time period
        } 
        else {
            // if pin B state changed before pin A, rotation is counter-clockwise
            bool_CW = false;
            encoderCount--;
            lastEncoderPeriod = period;
            speed -= (speed + 2.0*PI*rollerRadius*1000000.0/((float)encoderResolution*(float)period))*SpeedFilterAlpha; // Calculate speed in m/s based on encoder counts and time period
        }
        if (period > lastEncoderPeriod) {
            speed -= (speed - 2.0*PI*rollerRadius*1000000.0/((float)encoderResolution*(float)period)); // If period exceeds max allowed
        }
        encoderPrevTime = currentTime;
    }
    encoderPinA_prev = encoderPinA_value;
}

/// @brief Update current measurements from INA219 sensors and apply a low-pass filter to smooth the readings.
void updateMeasurements() {
    currentMeasurement_Spool -= (currentMeasurement_Spool - constrain(ina219_spool.getCurrent_mA(),0,1000))*CurrentfilterAlpha; // Simple low-pass filter to smooth current measurement for spool motor
    currentMeasurement_Roller -= (currentMeasurement_Roller - constrain(ina219_roller.getCurrent_mA(),0,1000))*CurrentfilterAlpha; // Simple low-pass filter to smooth current measurement for roller motor
}
