//Stepper motor control for the filament guide on a leadscrew
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_AS5600.h>
#include <AccelStepper.h>
#include "pid.h"
#include "driver/pcnt.h"

//two current sensors
Adafruit_INA219 ina219_spool(0x41);
Adafruit_AS5600 as5600;
//Adafruit_INA219 ina219_roller(0x40);
//PID SpoolPID(0.05, 0.15, 0.0); // Initialize PID controller with example gains
PID SpoolPID(7000, 2500, 5.0);
//PID RollerPID(1000, 0.3, 0.0); // Initialize PID controller with example gains
//PID RollerPID(12000, 6000, 300); // for 24 VDC motor
PID RollerPID(5000, 1500, 5.0); // for 12v dc motor
//PID RollerPID(10000, 3000, 5.0); // for 12v dc motor

// Board constants
#define ADC_bits 12.0        // ESP32 has 12-bit ADC
#define DAC_bits 8.0
const static float ADC_maxValue = pow(2, ADC_bits) - 1; // 4095 for 12-bit ADC
const static float DAC_maxValue = pow(2, DAC_bits) - 1; // 255 for 8-bit DAC

/* // ESP32 pin setup (ESP-WROOM-32 + CP2102 dev board)
#define motorRollerPin 18
#define motorSpoolPin 19
#define stepPin 25
#define dirPin 26
#define limitSwitchLowPin 27      // Normally LOW
#define limitSwitchHighPin 14     // Normally HIGH
#define encoderPinA 4             // CLK pin (PCNT-capable)
#define encoderPinB 5             // DT pin (PCNT-capable)
#define potPin 34                 // ADC1 input
#define potCurrentPin 35          // ADC1 input */

// ESP32 pin setup (ESP-WROOM-32 + CP2102 dev board)
#define motorRollerPin 25 //***
#define motorSpoolPin 26 //***
#define stepPin 2 //***
#define dirPin 15 //***
#define limitSwitchLowPin 35      // Normally LOW //***
#define limitSwitchHighPin 34     // Normally HIGH //***
#define encoderPinA 27             // CLK pin (PCNT-capable) //***
#define encoderPinB 14             // DT pin (PCNT-capable) //***
//#define potPin 4                 // ADC1 input //***
//#define potCurrentPin 33          // ADC1 input //***
#define switchManualPin 36 // manual control mode //***
#define switchLoadPin 39 // load control mode //***
#define fanControlPin 33 // fan control pin (PWM) //*** 
#define fanRSpeedPin 32 // fan RPM measurement pin //***
#define extruderSrewPin 23 // Extruder screw //this needs to be tested
#define led1Pin 19 // Led 1 //*** 
#define led2Pin 18 // Led 2 //*** 
#define led3Pin 4 // Led 3 //***

#define stepsPerRev 200.0 // full steps per revolution (NEMA17)
#define microsteps 16.0 // microsteps per full step (DRV8825 1/32)

#define leadScrewPitch 0.002 //m per revolution
#define ApproachStepInterval 1000.0 // microseconds between steps

#define guideMaxPosition 0.04 //m, maximum position of the filament guide
#define spoolRadius 0.053 //m, start radius of the filament spool
#define filamentDiameter 0.00285 //m, diameter of the filament
#define spoolWidth 0.045 //m, width of the filament spool

#define rollerRadius 0.0119 //m, radius of the roller in contact with the filament, used for speed calculation
#define encoderEdgesPerPulse 2.0 // encoderPinA interrupt on CHANGE counts both rising and falling edges

//#define encoderResolution 30.0 //pulses per revolution for the 
#define encoderResolution 600.0 //pulses per revolution for the encoder, used for speed calculation

// Filter variables for measurment smoothing
#define CurrentfilterAlpha 0.02 // Smoothing factor for current measurements
#define SpeedFilterAlpha 0.03 // Smoothing factor for speed measurements

#define PCNT_UNIT_USED PCNT_UNIT_0
#define FAN_PCNT_UNIT PCNT_UNIT_1
#define PCNT_H_LIM 32767

void IRAM_ATTR StepperLimit();
void initPCNT();
void pollPcntPulseEvent();
void processSerialInput();
void parseSerialSetpoints(char* input);
void initFanControlAndTach();
void setFanDutyPercent(uint8_t percent);
void updateFanRpm();
void enterLoadMode();
void exitLoadMode();
void updateLoadMode(unsigned long microsCurrent);
void syncGuidePositionFromStepper();

AccelStepper guideStepper(AccelStepper::DRIVER, stepPin, dirPin);


const static double stepsPerRevolution = stepsPerRev*microsteps;
const static double ratio = filamentDiameter/leadScrewPitch; //gear ratio between spool and guide, set to 1 for direct drive

volatile double guidePosition = 0.0; //m, current position of the filament guide
volatile int layerNumber = 0; //current layer number, used for testing

//Test parameters   
float targetSpeed = 0.02; //m/s, desired filament speed
double SetTorqueCurrent = 50; //mA, example torque setpoint for testing

// Calibration variables
float noLoadCurrent_Spool = 0.0; //mA, base no-load current for spool motor
float SpoolMotorCurrent = 0.0; //mA, current measurement for spool motor

// Stepper timing variables
volatile int stepDirection = HIGH;
unsigned long microsPrevStep = 0;

double speed = 0.0; //m/s, current speed of the filament, calculated from encoder counts
double traveledDistance = 0.0;    // m, total distance traveled by the filament, calculated by integrating speed over time
volatile double stepperSpeed = 0.0; // steps per second, for diagnostics

//Testing new speed measurement
volatile long encoderTicks = 0;
volatile uint8_t lastAState = 0;
int16_t prevPcntCount = 0;

// Calibration variables
const double speedCal = 500.0 / 703.23f; 

// timing variable
unsigned long lastDiagnoseTime = 0;
unsigned long lastEncoderPeriod = 0;
unsigned long encoderPrevTime = 0;
volatile unsigned long limit_triggered = 0;
bool interruptAttached = true; // Track interrupt attachment state
volatile bool limitSwitchEvent = false;
volatile bool encoderPulseEvent = false;
bool hasValidPulse = false;

// AS5600 magnetometer speed estimate (same shaft as roller)
bool as5600Available = false;
bool as5600InitPrinted = false;
uint16_t prevAs5600Angle = 0;
unsigned long prevAs5600Time = 0;
double magnetometerSpeed = 0.0; // m/s
double fanRpm = 0.0; // RPM
uint8_t fanDutyPercent = 40; // percent, 0..100

// Loop task periods (ms). Keep stepperControl in the fast path every iteration.
const unsigned long MEASUREMENT_PERIOD_MS = 5;
const unsigned long MAGNETOMETER_PERIOD_MS = 20;
const unsigned long DISTANCE_PERIOD_MS = 20;
const unsigned long SERIAL_INPUT_PERIOD_MS = 20;
const unsigned long MOTOR_CTRL_PERIOD_MS = 10;
const unsigned long DIAGNOSE_CALL_PERIOD_MS = 20;
const unsigned long FAN_MEASUREMENT_PERIOD_MS = 1000;
const unsigned long STOP_TIMEOUT_MS = 300;

// Ignore tiny near-zero speed noise when integrating traveled distance.
const float SPEED_DEADBAND_MPS = 0.0002f; // 0.2 mm/s
const float STEPPER_SPEED_FILTER_ALPHA = 0.08f; // Stronger smoothing to reduce audible jitter
const float STEPPER_MIN_SPEED_MPS = 0.0010f;    // Below this, stop stepping to avoid dithering noise
const float STEPPER_ACCEL_LIMIT_STEPS_S2 = 25000.0f; // Stepper speed slew-rate limit
unsigned long lastDistanceUpdateMs = 0;
char serialInputBuffer[64];
uint8_t serialInputIndex = 0;

// Fixed direction used for load-mode homing toward the low/home limit switch.
const int LOAD_HOME_DIRECTION = LOW;

// Use the same stepper speed formulation as standalone stepperControl test during load homing.
const double LOAD_HOME_FILAMENT_SPEED_MPS = 0.5;

double filteredFilamentSpeed = 0.0;
double commandedStepperSpeed = 0.0;
unsigned long lastStepperCtrlUs = 0;
long lastStepperPosSteps = 0;

enum LoadState {
    LOAD_IDLE = 0,
    LOAD_HOMING,
    LOAD_WAIT_PHASE
};

LoadState loadState = LOAD_IDLE;

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

    Serial.begin(115200);
    delay(500);
    
    Serial.println("Starting filament guide control system...");
    pinSetup();
    Serial.println("Pins initialized.");

    guideStepper.setMinPulseWidth(2);
    guideStepper.setMaxSpeed(20000.0);
    guideStepper.setAcceleration(40000.0);
    guideStepper.setSpeed(0.0);
    guideStepper.setCurrentPosition(0);
    lastStepperPosSteps = 0;

    // On reset, force normal mode to move toward the low/home limit first.
    stepDirection = LOAD_HOME_DIRECTION;
    guidePosition = guideMaxPosition;
    digitalWrite(dirPin, stepDirection);

    initPCNT();
    Serial.println("PCNT initialized.");
    initFanControlAndTach();
    Serial.println("Fan control initialized.");
    Serial.println("Initializing I2C peripherals...");
    initI2CPeripherals();
    Serial.println("I2C peripherals initialized.");
    Serial.println("Serial setpoint input enabled.");
    Serial.println("Use: S=20 (mm/s), C=50 (mA), F=40 (%), 20,50 or 20,50,40");
    
    // Set output limits for PID controllers
    SpoolPID.setOutputLimits(0, DAC_maxValue); // Set output limits for spool motor control
    //12V motor
    RollerPID.setOutputLimits(0, DAC_maxValue*0.6); // Set output limits for roller motor control

    // Run calibration 
    //HomingAndCalibration(5000, 100); // Run calibration for 5 seconds per motor, sampling every 100 ms

    //New speed mesurement test
    delay(100); // Short delay to ensure everything is initialized before starting speed measurement
    pcnt_get_counter_value(PCNT_UNIT_USED, &prevPcntCount);
    encoderPrevTime = millis(); // Initialize encoder timestamp to current time
    lastEncoderPeriod = 100; // Initialize to reasonable default period (~10 Hz)
    prevAs5600Time = millis();
    lastDistanceUpdateMs = millis();
}

void loop() {
    unsigned long currentMillis = millis();
    unsigned long microsCurrent = micros();
    bool loadSwitchActive = (digitalRead(switchLoadPin) == LOW); // active-low switch: ON -> GND

    static unsigned long lastMeasurementMs = 0;
    static unsigned long lastMagnetometerMs = 0;
    static unsigned long lastDistanceMs = 0;
    static unsigned long lastSerialInputMs = 0;
    static unsigned long lastMotorCtrlMs = 0;
    static unsigned long lastFanMeasurementMs = 0;
    static unsigned long lastDiagnoseCallMs = 0;

    if (loadSwitchActive && loadState == LOAD_IDLE) {
        enterLoadMode();
    } else if (!loadSwitchActive && loadState != LOAD_IDLE) {
        exitLoadMode();
    }

    if (loadState != LOAD_IDLE) {
        updateLoadMode(microsCurrent);
    } else {
        // Keep step generation in the fastest path for minimum timing jitter.
        stepperControl(microsCurrent, targetSpeed);
        //stepperControl(microsCurrent, 0.5); // Test with a constant speed command to verify stepper control independently of speed measurement
    }

    pollPcntPulseEvent();

    if (limitSwitchEvent && loadState == LOAD_IDLE) {
        noInterrupts();
        limitSwitchEvent = false;
        interrupts();

        if (digitalRead(limitSwitchLowPin) == HIGH) {
            stepDirection = HIGH;
            guidePosition = 0.0;
            layerNumber++;
            digitalWrite(dirPin, stepDirection);
            limit_triggered = currentMillis;
            detachInterrupt(digitalPinToInterrupt(limitSwitchHighPin));
            interruptAttached = false; // Mark interrupt as detached
        }
    }

    if (encoderPulseEvent) {
        noInterrupts();
        encoderPulseEvent = false;
        interrupts();
        updateSpeedByPulse();
    }

    if (currentMillis - lastMeasurementMs >= MEASUREMENT_PERIOD_MS) {
        updateMeasurements(); // Update Speed and Current measurements
        lastMeasurementMs = currentMillis;
    }

    if (currentMillis - lastMagnetometerMs >= MAGNETOMETER_PERIOD_MS) {
        measureMagnetometerSpeed(); // Keep AS5600 speed updated at a controlled rate
        lastMagnetometerMs = currentMillis;
    }

    if (currentMillis - lastDistanceMs >= DISTANCE_PERIOD_MS) {
        updateDistance(); // Integrate distance at a fixed interval like other tasks
        lastDistanceMs = currentMillis;
    }

    if (currentMillis - lastSerialInputMs >= SERIAL_INPUT_PERIOD_MS) {
        processSerialInput(); // Update setpoints from Serial Monitor input
        lastSerialInputMs = currentMillis;
    }

    if (currentMillis - lastMotorCtrlMs >= MOTOR_CTRL_PERIOD_MS && loadState == LOAD_IDLE) {
        motorControl(targetSpeed, speed);
        lastMotorCtrlMs = currentMillis;
    }

    if (currentMillis - lastFanMeasurementMs >= FAN_MEASUREMENT_PERIOD_MS) {
        updateFanRpm();
        lastFanMeasurementMs = currentMillis;
    }

    if (currentMillis - lastDiagnoseCallMs >= DIAGNOSE_CALL_PERIOD_MS) {
        diagnose(1000); // Print diagnostics every 1000 ms (1 second)
        lastDiagnoseCallMs = currentMillis;
    }

    // Re-attach interrupt after debounce period, but only if not already attached
    if (currentMillis - limit_triggered >= 500 && !interruptAttached){
        attachInterrupt(digitalPinToInterrupt(limitSwitchHighPin), StepperLimit, CHANGE);
        interruptAttached = true;
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
        */
        Serial.print("speed_target:");
        Serial.print(targetSpeed * 1000.0f);
        Serial.print(",measured_speed:");
        Serial.print(speed * 1000.0f);
        Serial.print(",distance mm:");
        Serial.print(traveledDistance * 1000.0f, 2);
        Serial.print(",fan_rpm:");
        Serial.print(fanRpm, 0);
        Serial.print(",fan_duty:");
        Serial.print(fanDutyPercent);
        Serial.print(",load_state:");
        Serial.print((int)loadState);
        //Serial.print(",mag_speed:");
        //Serial.print(magnetometerSpeed * 1000.0f);
        /* Serial.print(",target_current:");
        Serial.print(SetTorqueCurrent);
        Serial.print(",measured_current:");
        Serial.print(SpoolMotorCurrent); */
        /* Serial.print(",limit switch high pin:");
        Serial.print(digitalRead(limitSwitchHighPin));
        Serial.print(",limit switch low pin:");
        Serial.print(digitalRead(limitSwitchLowPin)); */
        /* Serial.print(",stpr pos: ");
        Serial.print(guidePosition);
        Serial.print(", Stp Dir: ");
        Serial.print(stepDirection);
        Serial.print(", Step speed: ");
        Serial.print(stepperSpeed); */

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
    RollerPID.setSetpoint(setSpeed); // Set speed setpoint for roller PID

    // Update PID controllers
    int controlSignal_Spool = SpoolPID.compute(torqueCurrent);
    int controlSignal_Roller = RollerPID.compute(actualSpeed);

    analogWrite(motorRollerPin, controlSignal_Roller);
    analogWrite(motorSpoolPin, controlSignal_Spool);
}

/// @brief 
/// @param microsCurrent //current time in microseconds for timing control of the stepper motor
/// @param filamentSpeed //desired filament speed in m/s, used to calculate the required stepper speed for the guide
void stepperControl(unsigned long microsCurrent, double filamentSpeed) {
    (void)microsCurrent;

    if (lastStepperCtrlUs == 0) {
        lastStepperCtrlUs = micros();
    }

    unsigned long nowUs = micros();
    double dt = (double)(nowUs - lastStepperCtrlUs) / 1000000.0;
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.05) dt = 0.05;
    lastStepperCtrlUs = nowUs;

    syncGuidePositionFromStepper();

    // Smooth noisy filament speed to reduce audible jitter.
    filteredFilamentSpeed += STEPPER_SPEED_FILTER_ALPHA * (filamentSpeed - filteredFilamentSpeed);

    double desiredStepperSpeed = 0.0;
    if (filteredFilamentSpeed > STEPPER_MIN_SPEED_MPS) {
        double spoolOmega = filteredFilamentSpeed / (spoolRadius + layerNumber * filamentDiameter);
        double guideOmega = spoolOmega * ratio;
        desiredStepperSpeed = guideOmega / (2.0 * PI) * stepsPerRevolution;
    }

    // Apply slew-rate limit so step frequency changes smoothly.
    double maxDelta = STEPPER_ACCEL_LIMIT_STEPS_S2 * dt;
    double speedError = desiredStepperSpeed - commandedStepperSpeed;
    if (speedError > maxDelta) speedError = maxDelta;
    if (speedError < -maxDelta) speedError = -maxDelta;
    commandedStepperSpeed += speedError;

    stepperSpeed = commandedStepperSpeed;
    if (stepperSpeed <= 0.0) {
        guideStepper.setSpeed(0.0);
        guideStepper.runSpeed();
        return;
    }

  // Count one layer when reaching max while moving forward, then reverse.
  if (guidePosition >= guideMaxPosition && stepDirection == HIGH) {
    guidePosition = guideMaxPosition;
    layerNumber++;
    stepDirection = LOW;
        long clampedSteps = (long)(guidePosition * stepsPerRevolution / leadScrewPitch);
        guideStepper.setCurrentPosition(clampedSteps);
        lastStepperPosSteps = clampedSteps;
  }

  // Count one layer when reaching zero while moving backward, then reverse.
  if (guidePosition <= 0.0 && stepDirection == LOW) {
    guidePosition = 0.0;
    layerNumber++;
    stepDirection = HIGH;
        guideStepper.setCurrentPosition(0);
        lastStepperPosSteps = 0;
  }

        double signedStepSpeed = (stepDirection == HIGH) ? stepperSpeed : -stepperSpeed;
        guideStepper.setSpeed(signedStepSpeed);
        guideStepper.runSpeed();
        syncGuidePositionFromStepper();
}

void enterLoadMode() {
    loadState = LOAD_HOMING;
    stepDirection = LOAD_HOME_DIRECTION; // drive guide toward low/home end
    digitalWrite(dirPin, stepDirection);
    digitalWrite(stepPin, LOW);
    microsPrevStep = micros();
    guideStepper.setSpeed(0.0);
    analogWrite(motorRollerPin, 0);
    analogWrite(motorSpoolPin, 0);
}

void exitLoadMode() {
    loadState = LOAD_IDLE;

    // Reset winding state when leaving load mode.
    traveledDistance = 0.0;
    guidePosition = 0.0;
    layerNumber = 0;
    stepDirection = HIGH; // move away from home/low limit when normal mode resumes
    digitalWrite(dirPin, stepDirection);

    digitalWrite(stepPin, LOW);
    analogWrite(motorRollerPin, 0);
    analogWrite(motorSpoolPin, 0);
}

void updateLoadMode(unsigned long microsCurrent) {
    switch (loadState) {
        case LOAD_HOMING:
            analogWrite(motorSpoolPin, 0); // spool stands still in load mode
            RollerPID.setSetpoint(targetSpeed);
            analogWrite(motorRollerPin, RollerPID.compute(speed)); // pulley keeps PID control

            // Keep homing direction pinned every cycle.
            stepDirection = LOAD_HOME_DIRECTION;
            digitalWrite(dirPin, stepDirection);

            // Same speed math style as stepperControl test, with fixed load homing speed input.
            {
                double spoolOmega = LOAD_HOME_FILAMENT_SPEED_MPS / (spoolRadius + layerNumber * filamentDiameter);
                double guideOmega = spoolOmega * ratio;
                stepperSpeed = guideOmega / (2.0 * PI) * stepsPerRevolution;
            }

            // Stop when low/home limit is reached.
            if (digitalRead(limitSwitchLowPin) == HIGH) {
                guidePosition = 0.0;
                digitalWrite(stepPin, LOW);
                loadState = LOAD_WAIT_PHASE;
                break;
            }

            if (stepperSpeed > 0.0) {
                double stepIntervalSeconds = 1.0 / stepperSpeed;
                unsigned long halfStepUs = (unsigned long)(1000000.0 * stepIntervalSeconds * 0.5);
                if (halfStepUs < 2) {
                    halfStepUs = 2;
                }

                // Catch up missed step toggles when loop timing is slower than target step rate.
                int maxCatchupToggles = 200;
                while ((microsCurrent - microsPrevStep >= halfStepUs) && (maxCatchupToggles-- > 0)) {
                    unsigned int state = digitalRead(stepPin);
                    unsigned int nextState = !state;
                    digitalWrite(stepPin, nextState);
                    microsPrevStep += halfStepUs;

                    // Update guide position only on STEP rising edge.
                    if (nextState == HIGH) {
                        if (stepDirection == HIGH) {
                            guidePosition += leadScrewPitch / stepsPerRevolution;
                        } else {
                            guidePosition -= leadScrewPitch / stepsPerRevolution;
                        }
                    }
                }
            }
            break;

        case LOAD_WAIT_PHASE:
            analogWrite(motorSpoolPin, 0); // spool stands still in load phase
            RollerPID.setSetpoint(targetSpeed);
            analogWrite(motorRollerPin, RollerPID.compute(speed)); // pulley keeps PID control while waiting
            break;

        case LOAD_IDLE:
        default:
            break;
    }
}

void syncGuidePositionFromStepper() {
    long currentPosSteps = guideStepper.currentPosition();
    long deltaSteps = currentPosSteps - lastStepperPosSteps;
    if (deltaSteps != 0) {
        guidePosition += ((double)deltaSteps) * (leadScrewPitch / stepsPerRevolution);
        lastStepperPosSteps = currentPosSteps;
    }
}

void IRAM_ATTR StepperLimit(){
    limitSwitchEvent = true;
}

void initPCNT() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = encoderPinA,
        .ctrl_gpio_num = encoderPinB,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = -PCNT_H_LIM,
        .unit = PCNT_UNIT_USED,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT_USED);
    pcnt_counter_clear(PCNT_UNIT_USED);
    pcnt_counter_resume(PCNT_UNIT_USED);
}

void pollPcntPulseEvent() {
    int16_t pcntCount = 0;
    pcnt_get_counter_value(PCNT_UNIT_USED, &pcntCount);

    int16_t countDelta = pcntCount - prevPcntCount;
    if (countDelta != 0) {
        encoderPulseEvent = true;
        prevPcntCount = pcntCount;
    }
}

void setFanDutyPercent(uint8_t percent) {
    if (percent > 100) percent = 100;
    fanDutyPercent = percent;
    int dutyRaw = map((int)fanDutyPercent, 0, 100, 0, (int)DAC_maxValue);
    ledcWrite(fanControlPin, dutyRaw);
}

void initFanControlAndTach() {
    // ESP32 Arduino Core v3 LEDC API
    ledcAttach(fanControlPin, 25000, 8);
    setFanDutyPercent(fanDutyPercent);

    pinMode(fanRSpeedPin, INPUT_PULLUP);

    pcnt_config_t fanPcntConfig = {
        .pulse_gpio_num = fanRSpeedPin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = PCNT_H_LIM,
        .counter_l_lim = 0,
        .unit = FAN_PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&fanPcntConfig);
    // Approx. 5 us digital glitch filter at 80 MHz APB clock -> 5e-6 * 80e6 = 400
    pcnt_set_filter_value(FAN_PCNT_UNIT, 400);
    pcnt_filter_enable(FAN_PCNT_UNIT);
    pcnt_counter_pause(FAN_PCNT_UNIT);
    pcnt_counter_clear(FAN_PCNT_UNIT);
    pcnt_counter_resume(FAN_PCNT_UNIT);
}

void updateFanRpm() {
    int16_t pulseCount = 0;
    pcnt_get_counter_value(FAN_PCNT_UNIT, &pulseCount);
    pcnt_counter_clear(FAN_PCNT_UNIT);

    fanRpm = ((float)pulseCount * 60.0f) / 2.0f; // 2 pulses/rev fan tach
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
    pinMode(limitSwitchLowPin, INPUT);
    pinMode(limitSwitchHighPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(limitSwitchHighPin), StepperLimit, CHANGE);

    pinMode(motorRollerPin, OUTPUT);
    pinMode(motorSpoolPin, OUTPUT);

    pinMode (encoderPinA, INPUT);
    pinMode (encoderPinB, INPUT);
    pinMode(switchManualPin, INPUT); // external pull-up expected, switch pulls to GND
    pinMode(switchLoadPin, INPUT);   // external pull-up expected, switch pulls to GND
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
                digitalWrite(dirPin, stepDirection); // Set direction for normal operation
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
    rawSpeed *= speedCal;
    speed += SpeedFilterAlpha * (rawSpeed - speed); // Apply low-pass filter to smooth speed measurement
    
    lastEncoderPeriod = period;
    encoderPrevTime = currentTime;
    hasValidPulse = true;
}

void decaySpeed() {
    unsigned long currentTime = millis();
    unsigned long gap_period = currentTime - encoderPrevTime; // Time since last pulse in milliseconds

    if (!hasValidPulse) {
        speed = 0.0;
        return;
    }

    if (gap_period > STOP_TIMEOUT_MS) {
        speed = 0.0;
        return;
    }

    if (gap_period > lastEncoderPeriod){ // Wait for a full period to elapse before decaying speed, ensures we only decay after missing a pulse
    float revPerSec = 1000.0 / ((float)gap_period * encoderResolution * encoderEdgesPerPulse); // Match encoder edge counting in ISR
    float rawSpeed = revPerSec * (2.0 * PI * rollerRadius); // Calculate raw speed in m/s based on roller radius
    rawSpeed *= speedCal;
    speed += SpeedFilterAlpha * (rawSpeed - speed); // Apply low-pass filter to smooth speed measurement
    }

    if (fabs(speed) < SPEED_DEADBAND_MPS) {
        speed = 0.0;
    }
}

// ---------------- Distance ----------------
void updateDistance() {
    unsigned long nowMs = millis();
    unsigned long dtMs = nowMs - lastDistanceUpdateMs;
    if (dtMs == 0) return;

    double dt = (double)dtMs / 1000.0;
    if (fabs(speed) >= SPEED_DEADBAND_MPS) {
        traveledDistance += speed * dt;
    }
    lastDistanceUpdateMs = nowMs;
}

/// @brief Update current measurements from INA219 sensors and apply a low-pass filter to smooth the readings.
void updateMeasurements() {
    float currentSpool = ina219_spool.getCurrent_mA();
    SpoolMotorCurrent += CurrentfilterAlpha*(currentSpool - SpoolMotorCurrent); // Simple low-pass filter to smooth current measurement for spool motor
    //updateSpeedEstimate();
    //TEST this:
    decaySpeed(); // Decay speed estimate if no pulses received, should be called regularly in loop
}

void processSerialInput() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            serialInputBuffer[serialInputIndex] = '\0';
            if (serialInputIndex > 0) {
                parseSerialSetpoints(serialInputBuffer);
            }
            serialInputIndex = 0;
            continue;
        }

        if (serialInputIndex < sizeof(serialInputBuffer) - 1) {
            serialInputBuffer[serialInputIndex++] = c;
        } else {
            serialInputIndex = 0;
        }
    }
}

void parseSerialSetpoints(char* input) {
    if (strcmp(input, "help") == 0) {
        Serial.println("Commands: S=20 (mm/s), C=50 (mA), F=40 (%), 20,50 or 20,50,40");
        return;
    }

    char* comma1 = strchr(input, ',');
    if (comma1 != nullptr) {
        *comma1 = '\0';
        float speedInMmS = atof(input);
        char* secondPart = comma1 + 1;
        char* comma2 = strchr(secondPart, ',');

        float currentIn = 0.0f;
        int fanDutyIn = fanDutyPercent;

        if (comma2 != nullptr) {
            *comma2 = '\0';
            currentIn = atof(secondPart);
            fanDutyIn = atoi(comma2 + 1);
        } else {
            currentIn = atof(secondPart);
        }

        float speedInMS = constrain(speedInMmS, 0.0f, 30.0f) / 1000.0f;
        targetSpeed = speedInMS;
        SetTorqueCurrent = constrain(currentIn, 0.0f, 200.0f);
        setFanDutyPercent((uint8_t)constrain(fanDutyIn, 0, 100));

        Serial.print("Updated setpoints -> speed(mm/s):");
        Serial.print(targetSpeed * 1000.0f, 2);
        Serial.print(", current(mA):");
        Serial.print(SetTorqueCurrent, 1);
        Serial.print(", fan(%):");
        Serial.println(fanDutyPercent);
        return;
    }

    if (input[0] == 'S' || input[0] == 's') {
        char* valuePtr = input + 1;
        if (*valuePtr == '=') valuePtr++;
        float speedInMmS = atof(valuePtr);
        float speedInMS = constrain(speedInMmS, 0.0f, 30.0f) / 1000.0f;
        targetSpeed = speedInMS;
        Serial.print("Updated speed setpoint (mm/s): ");
        Serial.println(targetSpeed * 1000.0f, 2);
        return;
    }

    if (input[0] == 'C' || input[0] == 'c') {
        char* valuePtr = input + 1;
        if (*valuePtr == '=') valuePtr++;
        float currentIn = atof(valuePtr);
        SetTorqueCurrent = constrain(currentIn, 0.0f, 200.0f);
        Serial.print("Updated current setpoint (mA): ");
        Serial.println(SetTorqueCurrent, 1);
        return;
    }

    if (input[0] == 'F' || input[0] == 'f') {
        char* valuePtr = input + 1;
        if (*valuePtr == '=') valuePtr++;
        int fanDutyIn = atoi(valuePtr);
        setFanDutyPercent((uint8_t)constrain(fanDutyIn, 0, 100));
        Serial.print("Updated fan duty (%): ");
        Serial.println(fanDutyPercent);
        return;
    }

    Serial.println("Invalid command. Use help, S=20, C=50, F=40, 20,50 or 20,50,40");
}

// Encoder pulse events are sourced from PCNT polling in pollPcntPulseEvent().

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