//Stepper motor control for the filament guide on a leadscrew
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include "pid.h"
#include "driver/pulse_cnt.h"

//two current sensors
Adafruit_INA219 ina219_spool(0x41);
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
#define switchManualPin 36 // manual control mode //***
#define switchLoadPin 39 // load control mode //***
#define fanControlPin 33 // fan control pin (PWM) //*** 
#define fanRSpeedPin 32 // fan RPM measurement pin //***
#define extruderSrewPin 23 // Extruder screw //this needs to be tested
#define led1Pin 19 // Led 1 //*** 
#define led2Pin 18 // Led 2 //*** 
#define led3Pin 4 // Led 3 //***

// TMC2209 UART configuration for guide stepper drivers
#define tmcRx1Pin 5
#define tmcTx1Pin 13
#define tmcBaud 115200

#define stepsPerRev 200.0 // full steps per revolution (NEMA17)
const float microsteps = 16.0f; // microsteps per full step

const uint16_t GUIDE_TMC_RUN_CURRENT_MA = 1000;
const uint16_t GUIDE_TMC_HOLD_CURRENT_MA = 200;
const uint16_t GUIDE_TMC_MICROSTEPS = (uint16_t)(microsteps + 0.5f);
const float GUIDE_TMC_R_SENSE = 0.11f;

#define leadScrewPitch 0.004 //m per revolution (4 mm pitch)
#define ApproachStepInterval 1000.0 // microseconds between steps

#define guideMaxPosition 0.05 //m, 5 cm guide travel like standalone test
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
void initGuideTmcUart();
void setGuideDriverCurrent(uint16_t currentmA);
void updateGuideDriverStatus();

FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper* guideStepper = nullptr;
HardwareSerial TMCSerial(1);
TMC2209Stepper guideDriver(&TMCSerial, GUIDE_TMC_R_SENSE, 0);


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

const bool GUIDE_DIR_PIN_INVERTED = true; // Flip DIR polarity when motor phases are rewired.

// Stepper timing variables
bool guideMovingTowardMax = true;
unsigned long microsPrevStep = 0;

double speed = 0.0; //m/s, current speed of the filament, calculated from encoder counts
double traveledDistance = 0.0;    // m, total distance traveled by the filament, calculated by integrating speed over time
volatile double stepperSpeed = 0.0; // steps per second, for diagnostics

//Testing new speed measurement
volatile long encoderTicks = 0;
volatile uint8_t lastAState = 0;
int prevPcntCount = 0;

pcnt_unit_handle_t encoderPcntUnit = nullptr;
pcnt_channel_handle_t encoderPcntChannel = nullptr;
pcnt_unit_handle_t fanPcntUnit = nullptr;
pcnt_channel_handle_t fanPcntChannel = nullptr;
bool encoderPcntReady = false;
bool fanPcntReady = false;

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

double fanRpm = 0.0; // RPM
uint8_t fanDutyPercent = 40; // percent, 0..100

// Loop task periods (ms). Keep stepperControl in the fast path every iteration.
const unsigned long MEASUREMENT_PERIOD_MS = 40;
const unsigned long DISTANCE_PERIOD_MS = 1000;
const unsigned long SERIAL_INPUT_PERIOD_MS = 200;
const unsigned long MOTOR_CTRL_PERIOD_MS = 40;
const unsigned long DIAGNOSE_CALL_PERIOD_MS = 100;
const unsigned long FAN_MEASUREMENT_PERIOD_MS = 1000;
const unsigned long TMC_STATUS_PERIOD_MS = 200;
const unsigned long STOP_TIMEOUT_MS = 300;

// Ignore tiny near-zero speed noise when integrating traveled distance.
const float SPEED_DEADBAND_MPS = 0.0002f; // 0.2 mm/s
const float STEPPER_SPEED_FILTER_ALPHA = 0.08f; // Stronger smoothing to reduce audible jitter
const float STEPPER_MIN_SPEED_MPS = 0.0010f;    // Below this, stop stepping to avoid dithering noise
const float STEPPER_MAX_SPEED_STEPS_S = 10000.0f;
const float STEPPER_ACCEL_LIBRARY_STEPS_S2 = 10000.0f;
const float LOAD_HOME_ACCEL_LIBRARY_STEPS_S2 = STEPPER_ACCEL_LIBRARY_STEPS_S2;
unsigned long lastDistanceUpdateMs = 0;
char serialInputBuffer[64];
uint8_t serialInputIndex = 0;

// Load-mode homing drives toward the low/home limit switch.
const float LOAD_HOME_CONSTANT_SPEED_STEPS_S = STEPPER_MAX_SPEED_STEPS_S;

double filteredFilamentSpeed = 0.0;
long lastStepperPosSteps = 0;
long guideMaxPositionSteps = 0;

enum LoadState {
    LOAD_IDLE = 0,
    LOAD_HOMING,
    LOAD_WAIT_PHASE
};

LoadState loadState = LOAD_IDLE;
bool guideDriverUartReady = false;
uint16_t guideDriverCurrentmA = 0;
uint32_t guideDriverStatusRaw = 0;
bool guideDriverOtpw = false;
bool guideDriverOt = false;


void setup() {

    Serial.begin(115200);
    delay(500);
    
    Serial.println("Starting filament guide control system...");
    pinSetup();
    Serial.println("Pins initialized.");

    stepperEngine.init();
    guideStepper = stepperEngine.stepperConnectToPin(stepPin);
    if (guideStepper == nullptr) {
        Serial.println("ERROR: guide stepper attach failed");
        while (1) {
            delay(10);
        }
    }

    guideStepper->setDirectionPin(dirPin, !GUIDE_DIR_PIN_INVERTED);
    guideStepper->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED_STEPS_S);
    guideStepper->setAcceleration((uint32_t)STEPPER_ACCEL_LIBRARY_STEPS_S2);
    guideMaxPositionSteps = (long)(guideMaxPosition * stepsPerRevolution / leadScrewPitch + 0.5);
    guideStepper->forceStopAndNewPosition((int32_t)guideMaxPositionSteps);
    lastStepperPosSteps = guideMaxPositionSteps;

    initGuideTmcUart();
    setGuideDriverCurrent(GUIDE_TMC_RUN_CURRENT_MA);

    // On reset, force normal mode to move toward the low/home limit first.
    guideMovingTowardMax = true;
    guidePosition = guideMaxPosition;
    guideStepper->stopMove();

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
    if (encoderPcntReady) {
        int initialEncoderCount = 0;
        if (pcnt_unit_get_count(encoderPcntUnit, &initialEncoderCount) == ESP_OK) {
            prevPcntCount = initialEncoderCount;
        }
    }
    encoderPrevTime = millis(); // Initialize encoder timestamp to current time
    lastEncoderPeriod = 100; // Initialize to reasonable default period (~10 Hz)
    lastDistanceUpdateMs = millis();
}

void loop() {
    unsigned long currentMillis = millis();
    unsigned long microsCurrent = micros();
    bool loadSwitchActive = (digitalRead(switchLoadPin) == LOW); // active-low switch: ON -> GND

    static unsigned long lastMeasurementMs = 0;
    static unsigned long lastDistanceMs = 0;
    static unsigned long lastSerialInputMs = 0;
    static unsigned long lastMotorCtrlMs = 0;
    static unsigned long lastFanMeasurementMs = 0;
    static unsigned long lastDiagnoseCallMs = 0;
    static unsigned long lastTmcStatusMs = 0;

    if (loadSwitchActive && loadState == LOAD_IDLE) {
        enterLoadMode();
    } else if (!loadSwitchActive && loadState != LOAD_IDLE) {
        exitLoadMode();
    }

    if (loadState != LOAD_IDLE) {
        updateLoadMode(microsCurrent);
    } else {
        // Keep step generation in the fastest path for minimum timing jitter.
        stepperControl(microsCurrent, speed);
        //stepperControl(microsCurrent, 0.5); // Test with a constant speed command to verify stepper control independently of speed measurement
    }

    pollPcntPulseEvent();

    if (limitSwitchEvent && loadState == LOAD_IDLE) {
        noInterrupts();
        limitSwitchEvent = false;
        interrupts();

        if (digitalRead(limitSwitchLowPin) == HIGH) {
            guideMovingTowardMax = false;
            guidePosition = 0.0;
            layerNumber++;
            guideStepper->forceStopAndNewPosition(0);
            guideStepper->moveTo((int32_t)guideMaxPositionSteps);
            lastStepperPosSteps = 0;
            stepperSpeed = 0.0;
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
        updateMeasurements(loadState == LOAD_IDLE); // Keep speed decay active; skip INA219 current sampling during load mode
        lastMeasurementMs = currentMillis;
    }

    if (currentMillis - lastDistanceMs >= DISTANCE_PERIOD_MS && loadState == LOAD_IDLE) {
        updateDistance(); // Distance tracking is only needed during normal winding mode
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

/*     if (currentMillis - lastTmcStatusMs >= TMC_STATUS_PERIOD_MS) {
        updateGuideDriverStatus();
        lastTmcStatusMs = currentMillis;
    } */

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
        /* Serial.print("speed_target:");
        Serial.print(targetSpeed * 1000.0f); */
        Serial.print("measured_speed: ");
        Serial.print(speed * 1000.0f);
        /* Serial.print(",distance mm:");
        Serial.print(traveledDistance * 1000.0f, 2); */
        Serial.print(",fan_rpm: ");
        Serial.print(fanRpm, 0);
    /*     Serial.print(",fan_duty:");
        Serial.print(fanDutyPercent);
        Serial.print(",load_state:");
        Serial.print((int)loadState);
        Serial.print(",tmc_mA:");
        Serial.print(guideDriverCurrentmA); */
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
        Serial.print(", To max: ");
        Serial.print((int)guideMovingTowardMax);
        Serial.print(", Step speed: ");
        Serial.print(stepperSpeed); */
        Serial.print(",guide position cm: ");
        Serial.println(guidePosition * 100.0, 4);
        /* Serial.print(", tmc_mA:");
        Serial.print(guideDriverCurrentmA);
        Serial.print(", otpw:");
        Serial.print((int)guideDriverOtpw);
        Serial.print(", ot:");
        Serial.print((int)guideDriverOt);
        Serial.print(", drv_status:0x");
        Serial.println(); // Newline for serial plotter */
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

    syncGuidePositionFromStepper();

    // Smooth noisy filament speed to reduce audible jitter.
    filteredFilamentSpeed += STEPPER_SPEED_FILTER_ALPHA * (filamentSpeed - filteredFilamentSpeed);

    double desiredStepperSpeed = 0.0;
    if (filteredFilamentSpeed > STEPPER_MIN_SPEED_MPS) {
        double spoolOmega = filteredFilamentSpeed / (spoolRadius + layerNumber * filamentDiameter);
        double guideOmega = spoolOmega * ratio;
        desiredStepperSpeed = guideOmega / (2.0 * PI) * stepsPerRevolution;
    }

    double commandedAbsSpeed = fabs(desiredStepperSpeed);
    if (commandedAbsSpeed <= 0.0) {
        guideStepper->stopMove();
        stepperSpeed = 0.0;
        syncGuidePositionFromStepper();
        return;
    }

    uint32_t commandedHz = (uint32_t)constrain(commandedAbsSpeed, 0.0, (double)STEPPER_MAX_SPEED_STEPS_S);
    guideStepper->setAcceleration((uint32_t)STEPPER_ACCEL_LIBRARY_STEPS_S2);
    guideStepper->setSpeedInHz(commandedHz);

    // Standalone-style control: run to endpoint target, then flip endpoint.
    if (!guideStepper->isRunning()) {
        layerNumber++;

        long currentPos = (long)guideStepper->getCurrentPosition();
        if (currentPos <= 1) {
            // At/near home -> always go away from limit switch.
            guideStepper->moveTo((int32_t)guideMaxPositionSteps);
            guideMovingTowardMax = false;
        } else if (currentPos >= (guideMaxPositionSteps - 1)) {
            // At/near max -> go back toward home.
            guideStepper->moveTo(0);
            guideMovingTowardMax = true;
        } else if (guideMovingTowardMax) {
            guideStepper->moveTo((int32_t)guideMaxPositionSteps);
            guideMovingTowardMax = false;
        } else {
            guideStepper->moveTo(0);
            guideMovingTowardMax = true;
        }
    }

    stepperSpeed = (double)commandedHz;
    syncGuidePositionFromStepper();
}

void enterLoadMode() {
    loadState = LOAD_HOMING;
    setGuideDriverCurrent(GUIDE_TMC_RUN_CURRENT_MA);

    guideMovingTowardMax = false; // drive guide toward low/home end
    digitalWrite(stepPin, LOW);
    microsPrevStep = micros();
    guideStepper->setAcceleration((uint32_t)LOAD_HOME_ACCEL_LIBRARY_STEPS_S2);
    guideStepper->setSpeedInHz((uint32_t)LOAD_HOME_CONSTANT_SPEED_STEPS_S);
    // Constant-speed homing like standalone behavior: run until limit switch triggers.
    guideStepper->runForward();
    analogWrite(motorRollerPin, 0);
    analogWrite(motorSpoolPin, 0);
}

void exitLoadMode() {
    loadState = LOAD_IDLE;
    setGuideDriverCurrent(GUIDE_TMC_RUN_CURRENT_MA);

    // Reset winding state when leaving load mode.
    traveledDistance = 0.0;
    guidePosition = 0.0;
    layerNumber = 0;
    // Resume normal mode by moving away from the low/home limit first.
    guideMovingTowardMax = false;
    guideStepper->setAcceleration((uint32_t)STEPPER_ACCEL_LIBRARY_STEPS_S2);
    guideStepper->stopMove();
    guideStepper->forceStopAndNewPosition(0);
    guideStepper->moveTo((int32_t)guideMaxPositionSteps);
    lastStepperPosSteps = 0;
    stepperSpeed = 0.0;

    digitalWrite(stepPin, LOW);
    analogWrite(motorRollerPin, 0);
    analogWrite(motorSpoolPin, 0);
}

void updateLoadMode(unsigned long microsCurrent) {
    (void)microsCurrent;
    switch (loadState) {
        case LOAD_HOMING: {
            analogWrite(motorSpoolPin, 0); // spool stands still in load mode
            RollerPID.setSetpoint(targetSpeed);
            analogWrite(motorRollerPin, RollerPID.compute(speed)); // pulley keeps PID control

            // Constant-speed homing to the low/home switch.
            syncGuidePositionFromStepper();
            stepperSpeed = LOAD_HOME_CONSTANT_SPEED_STEPS_S;

            // Stop when low/home limit is reached.
            if (digitalRead(limitSwitchLowPin) == HIGH) {
                guidePosition = 0.0;
                digitalWrite(stepPin, LOW);
                guideStepper->stopMove();
                guideStepper->forceStopAndNewPosition(0);
                guideStepper->moveTo(0);
                lastStepperPosSteps = 0;
                stepperSpeed = 0.0;
                loadState = LOAD_WAIT_PHASE;
                break;
            }
            break;
        }

        case LOAD_WAIT_PHASE:
            setGuideDriverCurrent(GUIDE_TMC_HOLD_CURRENT_MA);
            analogWrite(motorSpoolPin, 0); // spool stands still in load phase
            RollerPID.setSetpoint(targetSpeed);
            analogWrite(motorRollerPin, RollerPID.compute(speed)); // pulley keeps PID control while waiting
            break;

        case LOAD_IDLE:
        default:
            break;
    }
}

void initGuideTmcUart() {
    TMCSerial.begin(tmcBaud, SERIAL_8N1, tmcRx1Pin, tmcTx1Pin);
    delay(50);

    guideDriver.begin();
    guideDriver.pdn_disable(true);
    guideDriver.mstep_reg_select(true);
    guideDriver.I_scale_analog(false);
    guideDriver.toff(5);
    guideDriver.blank_time(24);
    guideDriver.microsteps(GUIDE_TMC_MICROSTEPS);
    guideDriver.pwm_autoscale(true);

    uint8_t conn = guideDriver.test_connection();
    guideDriverUartReady = (conn == 0);

    Serial.print("Guide tmc2209 UART test_connection() = ");
    Serial.println(conn);
    if (!guideDriverUartReady) {
        Serial.println("WARN: Guide tmc2209 UART not responding. Current/microstep commands may be ignored.");
    }
}

void setGuideDriverCurrent(uint16_t currentmA) {
    if (guideDriverCurrentmA == currentmA) {
        return;
    }

    guideDriverCurrentmA = currentmA;
    if (!guideDriverUartReady) {
        return;
    }

    guideDriver.rms_current(currentmA);
}

void updateGuideDriverStatus() {
    if (!guideDriverUartReady) {
        return;
    }

    uint32_t status = guideDriver.DRV_STATUS();
    guideDriverStatusRaw = status;

    bool prevOtpw = guideDriverOtpw;
    bool prevOt = guideDriverOt;

    // tmc2209 DRV_STATUS bits: OTPW=26, OT=25
    guideDriverOtpw = ((status >> 26) & 0x1U) != 0U;
    guideDriverOt = ((status >> 25) & 0x1U) != 0U;

    if (guideDriverOt && !prevOt) {
        Serial.println("tmc2209 OT active: thermal shutdown.");
    } else if (guideDriverOtpw && !prevOtpw) {
        Serial.println("tmc2209 OTPW active: near thermal shutdown.");
    }

    // Telemetry only: no automatic derating or motion intervention here.
}

void syncGuidePositionFromStepper() {
    if (guideStepper == nullptr) {
        return;
    }

    long currentPosSteps = (long)guideStepper->getCurrentPosition();
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
    encoderPcntReady = false;

    pcnt_unit_config_t encoderUnitConfig = {
        .low_limit = -PCNT_H_LIM,
        .high_limit = PCNT_H_LIM,
    };
    esp_err_t err = pcnt_new_unit(&encoderUnitConfig, &encoderPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_new_unit failed (%d)\n", (int)err);
        return;
    }

    pcnt_chan_config_t encoderChanConfig = {
        .edge_gpio_num = encoderPinA,
        .level_gpio_num = encoderPinB,
    };
    err = pcnt_new_channel(encoderPcntUnit, &encoderChanConfig, &encoderPcntChannel);
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_new_channel failed (%d)\n", (int)err);
        return;
    }

    err = pcnt_channel_set_edge_action(
        encoderPcntChannel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE
    );
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_channel_set_edge_action failed (%d)\n", (int)err);
        return;
    }

    err = pcnt_channel_set_level_action(
        encoderPcntChannel,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP
    );
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_channel_set_level_action failed (%d)\n", (int)err);
        return;
    }

    err = pcnt_unit_enable(encoderPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_unit_enable failed (%d)\n", (int)err);
        return;
    }
    err = pcnt_unit_clear_count(encoderPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_unit_clear_count failed (%d)\n", (int)err);
        return;
    }
    err = pcnt_unit_start(encoderPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_unit_start failed (%d)\n", (int)err);
        return;
    }

    encoderPcntReady = true;
}

void pollPcntPulseEvent() {
    if (!encoderPcntReady) {
        return;
    }

    int pcntCount = 0;
    if (pcnt_unit_get_count(encoderPcntUnit, &pcntCount) != ESP_OK) {
        return;
    }

    int countDelta = pcntCount - prevPcntCount;
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
    fanPcntReady = false;

    // ESP32 Arduino Core v3 LEDC API
    ledcAttach(fanControlPin, 25000, 8);
    setFanDutyPercent(fanDutyPercent);

    pinMode(fanRSpeedPin, INPUT_PULLUP);

    pcnt_unit_config_t fanUnitConfig = {
        .low_limit = -PCNT_H_LIM,
        .high_limit = PCNT_H_LIM,
    };
    esp_err_t err = pcnt_new_unit(&fanUnitConfig, &fanPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_new_unit failed (%d), fan RPM disabled\n", (int)err);
        return;
    }

    pcnt_chan_config_t fanChanConfig = {
        .edge_gpio_num = fanRSpeedPin,
        .level_gpio_num = -1,
    };
    err = pcnt_new_channel(fanPcntUnit, &fanChanConfig, &fanPcntChannel);
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_new_channel failed (%d), fan RPM disabled\n", (int)err);
        return;
    }

    err = pcnt_channel_set_edge_action(
        fanPcntChannel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD
    );
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_channel_set_edge_action failed (%d), fan RPM disabled\n", (int)err);
        return;
    }

    pcnt_glitch_filter_config_t fanFilterConfig = {
        .max_glitch_ns = 5000,
    };
    err = pcnt_unit_set_glitch_filter(fanPcntUnit, &fanFilterConfig);
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_unit_set_glitch_filter failed (%d), fan RPM disabled\n", (int)err);
        return;
    }

    err = pcnt_unit_enable(fanPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_unit_enable failed (%d), fan RPM disabled\n", (int)err);
        return;
    }
    err = pcnt_unit_clear_count(fanPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_unit_clear_count failed (%d), fan RPM disabled\n", (int)err);
        return;
    }
    err = pcnt_unit_start(fanPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("WARN: fan pcnt_unit_start failed (%d), fan RPM disabled\n", (int)err);
        return;
    }

    fanPcntReady = true;
}

void updateFanRpm() {
    if (!fanPcntReady) {
        fanRpm = 0.0;
        return;
    }

    int pulseCount = 0;
    if (pcnt_unit_get_count(fanPcntUnit, &pulseCount) != ESP_OK) {
        fanRpm = 0.0;
        return;
    }
    if (pcnt_unit_clear_count(fanPcntUnit) != ESP_OK) {
        fanRpm = 0.0;
        return;
    }

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

    bool calMovingTowardHome = true;
    auto setCalDirection = [&](bool towardHome) {
        uint8_t dirLevel = towardHome ? LOW : HIGH;
        if (GUIDE_DIR_PIN_INVERTED) {
            dirLevel = (dirLevel == HIGH) ? LOW : HIGH;
        }
        digitalWrite(dirPin, dirLevel);
    };

    setCalDirection(calMovingTowardHome); // Set initial direction towards the limit switch
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
                calMovingTowardHome = false;
                setCalDirection(calMovingTowardHome); // Set direction for normal operation
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

/// @brief Update speed/current measurements. In load mode, only speed decay is maintained.
void updateMeasurements(bool sampleCurrent) {
    if (sampleCurrent) {
        float currentSpool = ina219_spool.getCurrent_mA();
        SpoolMotorCurrent += CurrentfilterAlpha*(currentSpool - SpoolMotorCurrent); // Simple low-pass filter to smooth current measurement for spool motor
    }
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
