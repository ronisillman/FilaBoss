//Stepper motor control for the filament guide on a leadscrew
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_AS5600.h>
#include "pid.h"

//two current sensors
Adafruit_INA219 ina219_spool(0x41);
Adafruit_AS5600 as5600;
//Adafruit_INA219 ina219_roller(0x40);
//PID SpoolPID(0.05, 0.15, 0.0); // Initialize PID controller with example gains
PID SpoolPID(7000, 2500, 5.0);
//PID RollerPID(1000, 0.3, 0.0); // Initialize PID controller with example gains
//PID RollerPID(12000, 6000, 300); // for 24 VDC motor
PID RollerPID(5000, 1500, 5.0); // for 12v dc motor

// Board constants and variables
#define ADC_bits 10.0
#define DAC_bits 8.0
#define ADC_maxValue 1023.0
#define DAC_maxValue 255.0

//pin setup
#define motorRollerPin 5
#define motorSpoolPin 11
#define stepPin 9
#define dirPin 8
#define limitSwitchLowPin 7     //Normally LOw
#define limitSwitchHighPin 3 //Normally HIGH
#define encoderPinA 2 // CLK pin
#define encoderPinB 4 // DT pin
#define potPin A3 // Potentiometer for testing speed control
#define potCurrentPin A2 // Potentiometer for target current control

#define stepSize 20.0 //degrees per step
#define microsteps 32.0 //microsteps per full step
#define stepsPerRevolution 360.0*microsteps/stepSize
#define leadScrewPitch 0.005 //m per revolution
#define ApproachStepInterval 1000.0 // microseconds between steps

#define stepsPerMM stepsPerRevolution/leadScrewPitch

#define guideMaxPosition 0.05 //m, maximum position of the filament guide
#define spoolRadius 0.053 //m, start radius of the filament spool
#define filamentDiameter 0.00285 //m, diameter of the filament
#define spoolWidth 0.045 //m, width of the filament spool
#define ratio filamentDiameter/leadScrewPitch //gear ratio between spool and guide, set to 1 for direct drive
#define rollerRadius 0.0119 //m, radius of the roller in contact with the filament, used for speed calculation
#define encoderEdgesPerPulse 2.0 // encoderPinA interrupt on CHANGE counts both rising and falling edges

//#define encoderResolution 30.0 //pulses per revolution for the 
#define encoderResolution 600.0 //pulses per revolution for the encoder, used for speed calculation

// Filter variables for measurment smoothing
#define CurrentfilterAlpha 0.02 // Smoothing factor for current measurements
#define SpeedFilterAlpha 0.03 // Smoothing factor for speed measurements

double guidePosition = 0.0; //m, current position of the filament guide
int layerNumber = 0; //current layer number, used for testing

//Test parameters   
float targetSpeed = 0.02; //m/s, desired filament speed
double SetTorqueCurrent = 50; //mA, example torque setpoint for testing

// Calibration variables
float noLoadCurrent_Spool = 0.0; //mA, base no-load current for spool motor
float SpoolMotorCurrent = 0.0; //mA, current measurement for spool motor

// Stepper timing variables
int stepDirection = LOW;
unsigned long microsPrevStep = 0;

double speed = 0.0; //m/s, current speed of the filament, calculated from encoder counts

//Testing new speed measurement
volatile long encoderTicks = 0;
volatile uint8_t lastAState = 0;
unsigned long speedSamplePrevMs = 0;
long prevTicksForSpeed = 0;

// timing variable
unsigned long lastDiagnoseTime = 0;
unsigned long lastEncoderPeriod = 0;
unsigned long encoderPrevTime = 0;
unsigned long limit_triggered = 0;

// AS5600 magnetometer speed estimate (same shaft as roller)
bool as5600Available = false;
bool as5600InitPrinted = false;
uint16_t prevAs5600Angle = 0;
unsigned long prevAs5600Time = 0;
double magnetometerSpeed = 0.0; // m/s

// Input encoder variables
// Define rotary encoder pins
//#define ENC_A 2
//#define ENC_B 3
//unsigned long _lastIncReadTime = micros(); 
//unsigned long _lastDecReadTime = micros(); 
//#define _pauseLength 25000
//#define _fastIncrement 10
//volatile int counter = 0;


void setup() {

    Serial.begin(9600);
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Starting filament guide control system...");
    pinSetup();
    Serial.println("Pins initialized.");
    Serial.println("Initializing I2C peripherals...");
    initI2CPeripherals();
    Serial.println("I2C peripherals initialized.");
    
    // Set output limits for PID controllers
    SpoolPID.setOutputLimits(0, DAC_maxValue); // Set output limits for spool motor control
    //12V motor
    RollerPID.setOutputLimits(0, DAC_maxValue*0.6); // Set output limits for roller motor control

    // Run calibration 
    //HomingAndCalibration(5000, 100); // Run calibration for 5 seconds per motor, sampling every 100 ms

    //New speed mesurement test
    lastAState = digitalRead(encoderPinA);
    encoderPrevTime = millis(); // Initialize encoder timestamp to current time
    lastEncoderPeriod = 100; // Initialize to reasonable default period (~10 Hz)
    prevAs5600Time = millis();

}

void loop() {
    unsigned long currentMillis = millis();
    unsigned long microsCurrent = micros();

    updateMeasurements(); // Update Speed and Current measurements 
    measureMagnetometerSpeed(); // Keep AS5600 speed updated at loop rate

    stepperControl(microsCurrent, speed);
    potSpeedControl(); // Update target speed based on potentiometer reading
    potCurrentControl(); // Update target current based on potentiometer reading
    motorControl(targetSpeed, speed);
    diagnose(1000); // Print diagnostics every 1000 ms (1 second)
    if (currentMillis - limit_triggered >= 500){
    attachInterrupt(digitalPinToInterrupt(limitSwitchHighPin), StepperLimit, CHANGE);
    }
}

double measureMagnetometerSpeed() {
    if (!as5600Available) {
        magnetometerSpeed = 0.0;
        return magnetometerSpeed;
    }

    static unsigned long prevAs5600Micros = 0;
    const unsigned long sampleUs = 20000; // 20 ms fixed update for stable speed estimate
    const float maxAllowedRevPerSec = 3.0f; // reject unrealistic AS5600 spikes

    unsigned long nowUs = micros();
    if (prevAs5600Micros == 0) {
        prevAs5600Micros = nowUs;
        prevAs5600Angle = as5600.getAngle();
        return magnetometerSpeed;
    }

    unsigned long dtUs = nowUs - prevAs5600Micros;
    if (dtUs < sampleUs) {
        return magnetometerSpeed;
    }

    uint16_t angleNow = as5600.getAngle(); // 0..4095
    int16_t delta = (int16_t)angleNow - (int16_t)prevAs5600Angle;

    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    float dt = (float)dtUs / 1000000.0f;

    float maxDeltaCounts = maxAllowedRevPerSec * 4096.0f * dt;
    if (fabs((float)delta) > maxDeltaCounts) {
        prevAs5600Angle = angleNow;
        prevAs5600Micros = nowUs;
        return magnetometerSpeed;
    }

    float revPerSec = ((float)delta / 4096.0f) / dt;
    float rawSpeed = fabs(revPerSec * (2.0f * PI * rollerRadius));

    magnetometerSpeed += SpeedFilterAlpha * (rawSpeed - magnetometerSpeed);

    prevAs5600Angle = angleNow;
    prevAs5600Micros = nowUs;
    prevAs5600Time = millis();

    return magnetometerSpeed;
}

/// @brief Print diagnostic information to the serial monitor at a specified interval
/// @param interval The interval in milliseconds between diagnostic prints
void diagnose(unsigned long interval) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastDiagnoseTime >= interval) { // Print diagnostics every 1 second
        lastDiagnoseTime = currentMillis;
/*         Serial.print("Filament Speed(mm/s): ");
        Serial.print(speed*1000.0);
        Serial.print(" | Set Speed(mm/s): ");
        Serial.print(targetSpeed*1000.0);
        Serial.print(" | Spool Current(mA): ");
        Serial.print(SpoolMotorCurrent);
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
        // These four for PID plotting in the serial plotter
        
        Serial.print("speed_target:");
        Serial.print(targetSpeed * 1000.0f);
        Serial.print(",measured_speed:");
        Serial.print(speed * 1000.0f);
        Serial.print(",mag_speed:");
        Serial.print(magnetometerSpeed * 1000.0f);
        Serial.print(",target_current:");
        Serial.print(SetTorqueCurrent);
        Serial.print(",measured_current:");
        Serial.print(SpoolMotorCurrent);
        */
        Serial.print(",stpr pos: ");
        Serial.print(guidePosition);
        Serial.print(", Stp Dir: ");
        Serial.print(stepDirection);
        Serial.println(); // Newline for serial plotter
    }
}


/// @brief 
/// @param setSpeed Desired speed for the roller motor in m/s, used to calculate the speed error for the PID controller.
/// @param actualSpeed Actual speed of the roller motor in m/s, used to calculate the speed error for the PID controller.
/// TODO: Implement cascaded control to prevent spool and roller speed missmatch
void motorControl(float setSpeed, float actualSpeed) {
    // Calculate error from no-load current
    float torqueCurrent = SpoolMotorCurrent - noLoadCurrent_Spool*SpoolPID.getOutput()/DAC_maxValue; // Subtract scaled no-load current from actual current to get torque-related current for spool
    
    SpoolPID.setSetpoint(SetTorqueCurrent); // Set current setpoint for spool PID
    RollerPID.setSetpoint(setSpeed); // Set speed setpoint for roller PID (converted to mm/s for better resolution)
    // Update PID controllers
    int controlSignal_Spool = SpoolPID.compute(SpoolMotorCurrent);
    int controlSignal_Roller = RollerPID.compute(actualSpeed); // Compute control signal for roller PID (converted to mm/s for better resolution)

    analogWrite(motorRollerPin, controlSignal_Roller);
    analogWrite(motorSpoolPin, controlSignal_Spool);
}

/// @brief 
/// @param microsCurrent //current time in microseconds for timing control of the stepper motor
/// @param filamentSpeed //desired filament speed in m/s, used to calculate the required stepper speed for the guide
void stepperControl(unsigned long microsCurrent, double filamentSpeed) {
    double spoolOmega = filamentSpeed / (spoolRadius+layerNumber*filamentDiameter); // Calculate angular speed of the spool
    double guideOmega = spoolOmega * ratio;                                 // Calculate the required angular speed of the guide
    double stepperSpeed = guideOmega/(2.0 * PI) * stepsPerRevolution;       // Convert to steps per second
    double stepIntervalSeconds = 1.0 / stepperSpeed;                        // Calculate interval between steps in seconds

    if (guidePosition >= guideMaxPosition) {
        layerNumber++;
        stepDirection = LOW; // Reverse direction for next layer
        digitalWrite(dirPin, stepDirection); // Set direction
    }

    // Handle stepper motor movement (non-blocking)
    if (microsCurrent - microsPrevStep >= (unsigned long)((1000000.0*stepIntervalSeconds)/2.0)) { // Toggle step pin at half the interval for proper timing
        unsigned int state = digitalRead(stepPin);
        digitalWrite(stepPin, !state); // Toggle step pin
        microsPrevStep = microsCurrent;
        if (state == HIGH) { // Only count steps on the rising edge
            if (stepDirection == HIGH) {
                guidePosition += leadScrewPitch / stepsPerRevolution; // Update guide position
            } else {
                guidePosition -= leadScrewPitch / stepsPerRevolution; // Update guide position
            }
        }
    }
}

void StepperLimit(){
    if (digitalRead(limitSwitchLowPin) == HIGH){      
    Serial.print("limit switch triggered!");
    stepDirection = HIGH;
    guidePosition = 0.0;
    layerNumber++;
    digitalWrite(dirPin, stepDirection);
    limit_triggered = millis();
    detachInterrupt(limitSwitchHighPin);
    }
}

/// @brief Initialize I2C peripherals (current sensors) and check for their presence. If a sensor is not found, print an error message and halt execution.
void initI2CPeripherals() {
    Wire.begin();
  // Initialize INA219 at 0x41
  if (!ina219_spool.begin()) {
    Serial.println("Failed to find INA219 at address 0x41");
    while (1) {
      delay(10);
    }
  }

    as5600Available = as5600.begin();
    if (as5600Available) {
        prevAs5600Angle = as5600.getAngle();
        prevAs5600Time = millis();
        if (!as5600InitPrinted) {
            Serial.println("AS5600 initialized.");
            as5600InitPrinted = true;
        }
    } else if (!as5600InitPrinted) {
        Serial.println("AS5600 not found, mag speed disabled.");
        as5600InitPrinted = true;
    }
}

/// @brief Set pin modes for stepper control, limit switch, and motor control pins.
void pinSetup() {

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(limitSwitchLowPin, INPUT_PULLUP);
    pinMode(limitSwitchHighPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(limitSwitchHighPin), StepperLimit, CHANGE);

    pinMode(motorRollerPin, OUTPUT);
    pinMode(motorSpoolPin, OUTPUT);

    pinMode (encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
    pinMode (encoderPinB, INPUT_PULLUP);

    pinMode(potPin, INPUT);
    pinMode(potCurrentPin, INPUT);
    /*
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
    */
}

/// @brief Perform homing and no-load current calibration for the filament guide system.
/// @param calibrationTime_ms Duration in milliseconds for which to run the no-load current calibration. 
/// @param sampleInterval_ms Interval in milliseconds between current samples during calibration.
void HomingAndCalibration(int calibrationTime_ms, int sampleInterval_ms) {
    Serial.println("Starting no-load current calibration and guide homing...");    
    // Non-blocking calibration state and functions
    bool calibrated = true;
    bool homed = false;

    unsigned long calibStartTime = millis();
    unsigned long millisPrev = 0;

    float calib_spool_sum = 0.0;
    int calib_spool_samples = 0;

    stepDirection = LOW; // Set initial direction towards the limit switch
    digitalWrite(dirPin, stepDirection); // Set direction towards the limit switch
    analogWrite(motorSpoolPin, DAC_maxValue); // Run spool at full speed for calibration

    while (!calibrated || !homed) {
        unsigned long millisCurrent = millis();
        unsigned long microsCurrent = micros();

        if (!calibrated){
            // Sample current at regular intervals during calibration period
            if (millisCurrent - calibStartTime >= (unsigned long)calibrationTime_ms) {
                calibrated = true;
                noLoadCurrent_Spool = calib_spool_sum / calib_spool_samples;
                Serial.print("Spool no-load current: ");
                Serial.print(noLoadCurrent_Spool);
                Serial.println(" mA");
            }
            // Accumulate current samples for calibration
            if (millisCurrent - millisPrev >= (unsigned long)sampleInterval_ms) {
                calib_spool_sum += ina219_spool.getCurrent_mA();
                calib_spool_samples++;
                millisPrev = millisCurrent;
            }
        }
        // Handle homing towards the limit switch
        if (!homed) {
            if (digitalRead(limitSwitchHighPin) == LOW && digitalRead(limitSwitchLowPin) == HIGH) { // If limit switch is not triggered, continue homing
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
void updateSpeedByPulse() {
    unsigned long currentTime = millis();
    unsigned long period = currentTime - encoderPrevTime; // Time since last pulse in milliseconds
    if (period == 0) return; // Avoid division by zero, should not happen with proper timing
    // With CHANGE interrupt on channel A, we get 2 edges per encoder pulse.
    float revPerSec = 1000.0f / ((float)period * encoderResolution * encoderEdgesPerPulse); // revolutions per second
    float rawSpeed = revPerSec * (2.0f * PI * rollerRadius); // Calculate speed in m/s based on roller radius
    speed += SpeedFilterAlpha * (rawSpeed - speed); // Apply low-pass filter to smooth speed measurement
    
    lastEncoderPeriod = period;
    encoderPrevTime = currentTime;
}

void decaySpeed() {
    unsigned long currentTime = millis();
    unsigned long gap_period = currentTime - encoderPrevTime; // Time since last pulse in milliseconds

    if (gap_period > lastEncoderPeriod){ // Wait for a full period to elapse before decaying speed, ensures we only decay after missing a pulse
    float revPerSec = 1000.0 / ((float)gap_period * encoderResolution * encoderEdgesPerPulse); // Match encoder edge counting in ISR
    float rawSpeed = revPerSec * (2.0 * PI * rollerRadius); // Calculate raw speed in m/s based on roller radius
    speed += SpeedFilterAlpha * (rawSpeed - speed); // Apply low-pass filter to smooth speed measurement
    }
}

/// @brief Update current measurements from INA219 sensors and apply a low-pass filter to smooth the readings.
void updateMeasurements() {
    float currentSpool = ina219_spool.getCurrent_mA();
    SpoolMotorCurrent += CurrentfilterAlpha*(currentSpool - SpoolMotorCurrent); // Simple low-pass filter to smooth current measurement for spool motor
    //updateSpeedEstimate();
    //TEST this:
    decaySpeed(); // Decay speed estimate if no pulses received, should be called regularly in loop
}

void potSpeedControl() {
    float potValue = analogRead(potPin); // Read potentiometer value (0-1023)
    targetSpeed = ((ADC_maxValue - potValue) / ADC_maxValue) * 0.03; // Inverted map: 0.01-0.02 m/s
}

void potCurrentControl() {
    float potValue = analogRead(potCurrentPin); // Read potentiometer value (0-1023)
    SetTorqueCurrent = ((ADC_maxValue - potValue) / ADC_maxValue) * 200.0; // Inverted map: max pot -> min current
}

// New speed measurement test
void encoderISR() {
    uint8_t a = digitalRead(encoderPinA);
    uint8_t b = digitalRead(encoderPinB);

    if (a != lastAState) {
        encoderTicks += (b != a) ? 1 : -1;  // quadrature direction
        lastAState = a;

        //TEST this:
        updateSpeedByPulse(); // Update speed estimate on each pulse
    }
}

/* Based on Oleg Mazurov's code for rotary encoder interrupt service routines for AVR micros
   here https://chome.nerpa.tech/mcu/reading-rotary-encoder-on-arduino/
   and using interrupts https://chome.nerpa.tech/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/

   This example does not use the port read method. Tested with Nano and ESP32
   both encoder A and B pins must be connected to interrupt enabled pins, see here for more info:
   https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
} 
  */