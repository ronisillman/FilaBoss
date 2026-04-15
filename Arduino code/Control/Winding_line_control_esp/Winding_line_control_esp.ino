//Stepper motor control for the filament guide on a leadscrew
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <ArduinoJson.h>
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
PID RollerDiameterPID(5000, 1500, 5.0); // pulley PID for diameter mode (mm)
//PID RollerPID(10000, 3000, 5.0); // for 12v dc motor

// Board constants
#define ADC_bits 12.0        // ESP32 has 12-bit ADC
#define DAC_bits 8.0
const static float ADC_maxValue = pow(2, ADC_bits) - 1; // 4095 for 12-bit ADC
const static float DAC_maxValue = pow(2, DAC_bits) - 1; // 255 for 8-bit DAC

// ESP32 pin setup (ESP-WROOM-32 + CP2102 dev board)
#define motorRollerPin 12 //***
#define motorSpoolPin 13 //***
#define stepPin 4 //***
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
#define raspberryPiRXPIn 25 // UART RX pin for Raspberry Pi communication (if needed) //***
#define raspberryPiTXPin 26 // UART TX pin for Raspberry Pi communication (if needed) //***

// TMC2209 UART configuration for guide stepper drivers
#define tmcRx1Pin 16
#define tmcTx1Pin 17
#define tmcBaud 115200

#define stepsPerRev 200.0 // full steps per revolution (NEMA17)
const float microsteps = 16.0f; // microsteps per full step

const uint16_t GUIDE_TMC_RUN_CURRENT_MA = 1000;
const uint16_t GUIDE_TMC_HOLD_CURRENT_MA = 200;
const uint16_t GUIDE_TMC_MICROSTEPS = (uint16_t)(microsteps + 0.5f);
const float GUIDE_TMC_R_SENSE = 0.11f;

#define leadScrewPitch 0.004 //m per revolution (4 mm pitch)
#define ApproachStepInterval 1000.0 // microseconds between steps

#define guideMaxPosition 0.02 //m, 5 cm guide travel like standalone test
#define spoolRadius 0.053 //m, start radius of the filament spool
#define filamentDiameter 0.002 //m, diameter of the filament
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
void exitLoadMode(bool keepStopped = false);
void updateLoadMode(unsigned long microsCurrent);
void syncGuidePositionFromStepper();
void initGuideTmcUart();
void setGuideDriverCurrent(uint16_t currentmA);
void initRaspberrySerial();
void pollRaspberrySerial();
void processRaspberryJsonLine(const char* line);
bool parseRaspberryCommands(const char* line);
void applyRaspberryCommands();
void sendTelemetryToRaspberry();
//void updateGuideDriverStatus();
void initGuideStepper();

FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper* guideStepper = nullptr;
HardwareSerial TMCSerial(1);
HardwareSerial RaspberrySerial(2);
TMC2209Stepper guideDriver(&TMCSerial, GUIDE_TMC_R_SENSE, 0);

struct TelemetryFromEsp32 {
    bool load_mode;
    bool waiting_for_load;
    float filament_speed_mps;
    float fan_rpm;
    float diameter_travelled_mm;
    float spool_current_ma;
};

struct CommandsToEsp32 {
    float pid_p_pulley_dia;
    float pid_i_pulley_dia;
    float pid_d_pulley_dia;
    float pid_p_pulley_spd;
    float pid_i_pulley_spd;
    float pid_d_pulley_spd;
    float pid_p_spool;
    float pid_i_spool;
    float pid_d_spool;
    uint8_t fan_speed_pct;
    float spool_current_target_ma;
    char target_mode[4];
    float target_diameter_mm;
    float measured_diameter_mm;
    float target_speed_mps;
};


const static double stepsPerRevolution = stepsPerRev*microsteps;
const static double ratio = filamentDiameter/leadScrewPitch; //gear ratio between spool and guide, set to 1 for direct drive

volatile double guidePosition = 0.0; //m, current position of the filament guide
volatile int layerNumber = 0; //current layer number, used for testing

//Test parameters   
float targetSpeed = 0.01; //m/s, desired filament speed
double SetTorqueCurrent = 50; //mA, example torque setpoint for testing

// Calibration variables
float noLoadCurrent_Spool = 0.0; //mA, base no-load current for spool motor
float SpoolMotorCurrent = 0.0; //mA, current measurement for spool motor

const bool GUIDE_DIR_PIN_INVERTED = true; 

// Stepper timing variables
bool guideMovingTowardMax = false;
bool guideEndpointHandled = false;

// Wait for load phase on reset
bool waitingForLoad = true;
bool controlPaused = false;
double commandedAbsSpeed = 0.0; //m/s, absolute value of the desired filament speed, used for stepper speed control

double speed = 0.0; //m/s, current speed of the filament, calculated from encoder counts
double traveledDistance = 0.0;    // m, total distance traveled by the filament, calculated by integrating speed over time

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
//const double speedCal = 500.0 / 703.23f; 
const double speedCal = 500.0 / 715.23f;

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
uint8_t fanDutyPercent = 100; // percent, 0..100

// Loop task periods (ms). Keep stepperControl in the fast path every iteration.
const unsigned long MEASUREMENT_PERIOD_MS = 10;
const unsigned long DISTANCE_PERIOD_MS = 100;
const unsigned long SERIAL_INPUT_PERIOD_MS = 100;
const unsigned long MOTOR_CTRL_PERIOD_MS = 10;
const unsigned long POSITION_SYNC_PERIOD_MS = 20;
const unsigned long DIAGNOSE_CALL_PERIOD_MS = 1000;
const unsigned long FAN_MEASUREMENT_PERIOD_MS = 500;
const unsigned long TMC_STATUS_PERIOD_MS = 200;
const unsigned long STOP_TIMEOUT_MS = 300;
const unsigned long RASPBERRY_TELEMETRY_PERIOD_MS = 50;
const unsigned long RASPBERRY_COMMAND_TIMEOUT_MS = 1000;
const float FAN_TACH_PULSES_PER_REV = 2.0f;

// Ignore tiny near-zero speed noise when integrating traveled distance.
const float SPEED_DEADBAND_MPS = 0.0002f; // 0.2 mm/s
const float STEPPER_MAX_SPEED = 8000.0f;
const float STEPPER_MAX_ACCELERATION = 10000.0f;
unsigned long lastDistanceUpdateMs = 0;
char serialInputBuffer[64];
uint8_t serialInputIndex = 0;

char raspberrySerialBuffer[768];
uint16_t raspberrySerialIndex = 0;
unsigned long lastRaspberryCommandMs = 0;

CommandsToEsp32 raspberryCommands = {
    5000.0f,
    1500.0f,
    5.0f,
    5000.0f,
    1500.0f,
    5.0f,
    7000.0f,
    2500.0f,
    5.0f,
    100,
    50.0f,
    "Spd",
    1.75f,
    1.75f,
    0.01f,
};

long lastStepperPosSteps = 0;
long guideMaxPositionSteps = 0;
const double guideInitialPosition = 0.0055;
long guideInitialPosSteps = 0;

enum LoadState {
    LOAD_IDLE = 0,
    LOAD_HOMING,
    LOAD_MOVE_TO_INITIAL,
    LOAD_WAIT_PHASE
};

LoadState loadState = LOAD_IDLE;
bool guideDriverUartReady = false;
bool raspberrySerialReady = false;

uint16_t guideDriverCurrentmA = 0;
//uint32_t guideDriverStatusRaw = 0;
//bool guideDriverOtpw = false;
//bool guideDriverOt = false;


void setup() {

    Serial.begin(115200);
    delay(500);
    
    Serial.println("Starting filament guide control system...");
    pinSetup();
    Serial.println("Pins initialized.");
    initGuideTmcUart();
    Serial.println("TMC2209 UART initialized for guide driver.");
    initRaspberrySerial();
    Serial.println("Raspberry UART2 JSON link initialized.");
    initGuideStepper();
    Serial.println("Guide stepper initialized.");
    initPCNT();
    Serial.println("PCNT initialized.");
    initFanControlAndTach();
    Serial.println("Fan control initialized.");
    initI2CPeripherals();
    Serial.println("I2C peripherals initialized.");
    Serial.println("*****************************");
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
    if (waitingForLoad) {
        Serial.println("Initializing complete. Waiting for load switch activation to start normal operation...");
    }

    static unsigned long lastWaitingTelemetryMs = 0;
    while (waitingForLoad) {
        pollRaspberrySerial();
        unsigned long waitNowMs = millis();
        if (raspberrySerialReady && (waitNowMs - lastWaitingTelemetryMs >= RASPBERRY_TELEMETRY_PERIOD_MS)) {
            sendTelemetryToRaspberry();
            lastWaitingTelemetryMs = waitNowMs;
        }

        if (digitalRead(switchLoadPin) == LOW) {
            waitingForLoad = false;
        }
        delay(10);
    }

    bool extruderON = (digitalRead(extruderSrewPin) == LOW);
    bool manualSwitchON = (digitalRead(switchManualPin) == LOW);
    
    // Only react to extruder pin if manual switch is NOT engaged
    if (!extruderON && !manualSwitchON) {
            if (!controlPaused) {
                Serial.println("Extruder srew off. Pausing control until it is turned on or manual switch is engaged.");
                analogWrite(motorRollerPin, 0);
                analogWrite(motorSpoolPin, 0);
                if (guideStepper != nullptr) {
                    guideStepper->stopMove();
                }
            }
            controlPaused = true;
    } else {
        if (controlPaused) {
            // Use targetSpeed as fallback when measured speed is near zero
            // (pulley was stopped during pause) to avoid setSpeedInHz(0) being
            // ignored and the stepper resuming at whatever speed was last set.
            double resumeSpeed = (speed > SPEED_DEADBAND_MPS) ? speed : targetSpeed;
            if (guideMovingTowardMax && LOAD_IDLE == loadState) {
                commandedAbsSpeed = fabs(computeGuideSpeedHz(resumeSpeed));
                if (commandedAbsSpeed < 1.0) commandedAbsSpeed = 1.0;
                guideStepper->setSpeedInHz(commandedAbsSpeed);
                guideStepper->moveTo((int32_t)guideMaxPositionSteps);
            } else if (LOAD_IDLE == loadState) {
                commandedAbsSpeed = fabs(computeGuideSpeedHz(resumeSpeed));
                if (commandedAbsSpeed < 1.0) commandedAbsSpeed = 1.0;
                guideStepper->setSpeedInHz(commandedAbsSpeed);
                guideStepper->moveTo(0);
            }
            Serial.println("Continuing normal operation.");
        }
        controlPaused = false;
    }


    unsigned long currentMillis = millis();
    unsigned long microsCurrent = micros();
    bool loadSwitchActive = (digitalRead(switchLoadPin) == LOW); // active-low switch: ON -> GND

    static unsigned long lastMeasurementMs = 0;
    static unsigned long lastDistanceMs = 0;
    static unsigned long lastSerialInputMs = 0;
    static unsigned long lastMotorCtrlMs = 0;
    static unsigned long lastPositionSyncMs = 0;
    static unsigned long lastFanMeasurementMs = 0;
    static unsigned long lastDiagnoseCallMs = 0;
    static unsigned long lastTmcStatusMs = 0;
    static unsigned long lastRaspberryTelemetryMs = 0;

    // Uncomment to enable Raspberry UART2 JSON RX.
    pollRaspberrySerial();

    // Uncomment to enable command-timeout handling.
    if (raspberrySerialReady && (currentMillis - lastRaspberryCommandMs > RASPBERRY_COMMAND_TIMEOUT_MS)) {
    //     Keep last command by default. Add fail-safe behavior here if needed.
    }

    if (loadSwitchActive && loadState == LOAD_IDLE) {
        enterLoadMode();
    } else if (!loadSwitchActive && loadState != LOAD_IDLE) {
        exitLoadMode(controlPaused);
    }

    if (loadState != LOAD_IDLE) {
        updateLoadMode(microsCurrent);
    } else {
        if (!controlPaused) {
            stepperControl(microsCurrent, speed);
        }
    }

    if (limitSwitchEvent && loadState == LOAD_IDLE) {
        noInterrupts();
        limitSwitchEvent = false;
        interrupts();

        if (digitalRead(limitSwitchLowPin) == HIGH) {
            guideMovingTowardMax = true;
            guidePosition = 0.0;
            guideStepper->forceStopAndNewPosition(0);
            guideStepper->moveTo((int32_t)guideMaxPositionSteps);
            lastStepperPosSteps = 0;
            limit_triggered = currentMillis;
            detachInterrupt(digitalPinToInterrupt(limitSwitchHighPin));
            interruptAttached = false; // Mark interrupt as detached
        }
    }

    pollPcntPulseEvent();

    if (encoderPulseEvent) {
        noInterrupts();
        encoderPulseEvent = false;
        interrupts();
        updateSpeedByPulse();
    }

    if (currentMillis - lastMeasurementMs >= MEASUREMENT_PERIOD_MS) {
        updateMeasurements(); 
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

    if (currentMillis - lastMotorCtrlMs >= MOTOR_CTRL_PERIOD_MS && loadState == LOAD_IDLE && !controlPaused) {
        motorControl(targetSpeed, speed);
        lastMotorCtrlMs = currentMillis;
    }

    if (currentMillis - lastPositionSyncMs >= POSITION_SYNC_PERIOD_MS && loadState == LOAD_IDLE && !controlPaused) {
        syncGuidePositionFromStepper();
        lastPositionSyncMs = currentMillis;
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
        diagnose(); // Print diagnostics every 1000 ms (1 second)
        lastDiagnoseCallMs = currentMillis;
    }

    // Uncomment to enable Raspberry UART2 JSON TX telemetry.
    if (raspberrySerialReady && currentMillis - lastRaspberryTelemetryMs >= RASPBERRY_TELEMETRY_PERIOD_MS) {
        sendTelemetryToRaspberry();
        lastRaspberryTelemetryMs = currentMillis;
    }

    // Re-attach interrupt after debounce period, but only if not already attached
    if (currentMillis - limit_triggered >= 500 && !interruptAttached){
        attachInterrupt(digitalPinToInterrupt(limitSwitchHighPin), StepperLimit, CHANGE);
        interruptAttached = true;
    }
}

/// @brief Print diagnostic information to the serial monitor at a specified interval
/// @param interval The interval in milliseconds between diagnostic prints
void diagnose() {
    Serial.print(", Speed (mm/s): ");
    Serial.print(speed * 1000.0f, 2);
    Serial.print(", Target speed (mm/s): ");
    Serial.print(targetSpeed * 1000.0f, 2);
    Serial.print(", Spool current (mA): ");
    Serial.print(SpoolMotorCurrent, 2);
    Serial.print(", Spool current target (mA): ");
    Serial.print(SetTorqueCurrent, 2);
    Serial.print("Fan RPM: ");
    Serial.print(fanRpm, 1);
    Serial.println();
}


/// @brief 
/// @param setSpeed Desired speed for the roller motor in m/s, used to calculate the speed error for the PID controller.
/// @param actualSpeed Actual speed of the roller motor in m/s, used to calculate the speed error for the PID controller.
/// TODO: Implement cascaded control to prevent spool and roller speed missmatch
int computePulleyControlSignal(float setSpeed, float actualSpeed, bool forceSpeedMode = false) {
    if (!forceSpeedMode && strcmp(raspberryCommands.target_mode, "Dia") == 0) {
        float targetDiameter = constrain(raspberryCommands.target_diameter_mm, 0.5f, 5.0f);
        float measuredDiameter = constrain(raspberryCommands.measured_diameter_mm, 0.5f, 5.0f);
        RollerDiameterPID.setSetpoint(targetDiameter);
        return RollerDiameterPID.compute(measuredDiameter);
    }

    RollerPID.setSetpoint(setSpeed); // Speed mode uses measured filament speed feedback
    return RollerPID.compute(actualSpeed);
}

void motorControl(float setSpeed, float actualSpeed) {
    // Calculate error from no-load current
    float torqueCurrent = SpoolMotorCurrent - noLoadCurrent_Spool*SpoolPID.getOutput()/DAC_maxValue; // Subtract scaled no-load current from actual current to get torque-related current for spool
    
    SpoolPID.setSetpoint(SetTorqueCurrent); // Set current setpoint for spool PID

    // Update PID controllers
    int controlSignal_Spool = SpoolPID.compute(torqueCurrent);
    int controlSignal_Roller = computePulleyControlSignal(setSpeed, actualSpeed);

    analogWrite(motorRollerPin, controlSignal_Roller);
    analogWrite(motorSpoolPin, controlSignal_Spool);
}

double computeGuideSpeedHz(double filamentSpeedMps) {
    double spoolOmega = filamentSpeedMps / (spoolRadius + layerNumber * filamentDiameter);
    double guideOmega = spoolOmega * ratio;
    double hz = fabs(guideOmega / (2.0 * PI) * stepsPerRevolution);
    return hz;
}

/// @brief 
/// @param microsCurrent //current time in microseconds for timing control of the stepper motor
/// @param filamentSpeed //desired filament speed in m/s, used to calculate the required stepper speed for the guide
void stepperControl(unsigned long microsCurrent, double filamentSpeed) {
    (void)microsCurrent;

    commandedAbsSpeed = fabs(computeGuideSpeedHz(filamentSpeed));
    guideStepper->setSpeedInHz(commandedAbsSpeed);

    if (guideStepper->isRunning()) {
        guideEndpointHandled = false;
        return;
    }

    long currentPosition = guideStepper->getCurrentPosition();

    // Only count a layer when the guide has actually reached either endpoint.
    if (!guideEndpointHandled && (currentPosition == 0 || currentPosition == guideMaxPositionSteps)) {
        layerNumber++;
        guideEndpointHandled = true;

        if (currentPosition == guideMaxPositionSteps) {
            guideStepper->moveTo(0);
            guideMovingTowardMax = false;
        } else {
            guideStepper->moveTo((int32_t)guideMaxPositionSteps);
            guideMovingTowardMax = true;
        }
    }
}

void enterLoadMode() {
    loadState = LOAD_HOMING;
    setGuideDriverCurrent(1500); // Use higher current for homing to ensure the guide can reliably reach the limit switch
    guideStepper->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED);
    guideStepper->runForward();
    analogWrite(motorSpoolPin, 0); // spool stands still in load mode
    analogWrite(motorRollerPin, computePulleyControlSignal(targetSpeed, speed, true)); // load mode always uses speed PID for pulley
}

void exitLoadMode(bool keepStopped) {
    loadState = LOAD_IDLE;
    setGuideDriverCurrent(GUIDE_TMC_RUN_CURRENT_MA); // Restore normal running current for the guide
    // Reset winding state when leaving load mode.
    traveledDistance = 0.0;
    lastDistanceUpdateMs = millis();
    guidePosition = 0.0;
    layerNumber = 0;
    lastStepperPosSteps = 0;
    guideEndpointHandled = false;
    // Resume normal mode by moving away from the low/home limit first.
    guideMovingTowardMax = true;
    guideStepper->stopMove();
    guideStepper->forceStopAndNewPosition(0);
    if (!keepStopped) {
        // Explicitly set winding speed before starting the move. Without this,
        // the stepper retains STEPPER_MAX_SPEED from homing and runs at full
        // speed until stepperControl() corrects it — which only happens once a
        // valid encoder reading is available (~50% of exits had the bug).
        commandedAbsSpeed = fabs(computeGuideSpeedHz(targetSpeed));
        if (commandedAbsSpeed < 1.0) commandedAbsSpeed = 1.0;
        guideStepper->setSpeedInHz((uint32_t)commandedAbsSpeed);
        guideStepper->moveTo((int32_t)guideMaxPositionSteps);
    }

    analogWrite(motorRollerPin, 0);
    analogWrite(motorSpoolPin, 0);
}

void updateLoadMode(unsigned long microsCurrent) {
    (void)microsCurrent;
    switch (loadState) {
        case LOAD_HOMING: {
            analogWrite(motorSpoolPin, 0); // spool stands still in load mode
            analogWrite(motorRollerPin, computePulleyControlSignal(targetSpeed, speed, true)); // load mode always uses speed PID for pulley

            // Stop when low/home limit is reached.
            if (digitalRead(limitSwitchLowPin) == HIGH) {
                guideStepper->stopMove();
                guideStepper->forceStopAndNewPosition(0);
                guideStepper->moveTo((int32_t)guideInitialPosSteps);
                loadState = LOAD_MOVE_TO_INITIAL;
                break;
            }
            break;
        }

        case LOAD_MOVE_TO_INITIAL:
            analogWrite(motorSpoolPin, 0); // spool stands still in load phase
            analogWrite(motorRollerPin, computePulleyControlSignal(targetSpeed, speed, true)); // keep pulley control active while guide moves to initial point

            // Non-blocking move completion keeps UART2 polling/telemetry running.
            if (!guideStepper->isRunning()) {
                guideStepper->stopMove();
                guideStepper->forceStopAndNewPosition((int32_t)0);
                loadState = LOAD_WAIT_PHASE;
            }
            break;

        case LOAD_WAIT_PHASE:
            setGuideDriverCurrent(GUIDE_TMC_HOLD_CURRENT_MA);
            analogWrite(motorSpoolPin, 0); // spool stands still in load phase
            analogWrite(motorRollerPin, computePulleyControlSignal(targetSpeed, speed, true)); // load mode always uses speed PID for pulley
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

void initGuideStepper() {
    stepperEngine.init();
    guideStepper = stepperEngine.stepperConnectToPin(stepPin);
    if (guideStepper == nullptr) {
        Serial.println("ERROR: guide stepper attach failed");
        while (1) {
            delay(10);
        }
    }
    setGuideDriverCurrent(GUIDE_TMC_RUN_CURRENT_MA);
    guideStepper->setDirectionPin(dirPin, !GUIDE_DIR_PIN_INVERTED);
    guideStepper->setSpeedInHz((uint32_t)STEPPER_MAX_SPEED);
    guideStepper->setAcceleration((uint32_t)STEPPER_MAX_ACCELERATION);
    guideMaxPositionSteps = -(long)(guideMaxPosition * stepsPerRevolution / leadScrewPitch + 0.5);
    guideInitialPosSteps = -(long)(guideInitialPosition * stepsPerRevolution / leadScrewPitch + 0.5);

/*     // Home the guide against the low limit switch before normal operation starts.
    guideMovingTowardMax = false;
    if (digitalRead(limitSwitchLowPin) != HIGH) {
        guideStepper->runForward();

        while (digitalRead(limitSwitchLowPin) != HIGH) {
            delay(1);
        }
    }
    guideStepper->stopMove();
    guideStepper->forceStopAndNewPosition((int32_t)0);

    guideStepper->moveTo((int32_t)guideInitialPosSteps);
    while (guideStepper->isRunning()) {
        delay(1);
    }
    guideStepper->stopMove();
    guideStepper->forceStopAndNewPosition((int32_t)0);

    lastStepperPosSteps = 0;
    guidePosition = 0.0;
    layerNumber = 0; */

}

/* void updateGuideDriverStatus() {
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
} */

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

    // Convert counted pulses over FAN_MEASUREMENT_PERIOD_MS to RPM.
    fanRpm = ((float)pulseCount * (60000.0f / (float)FAN_MEASUREMENT_PERIOD_MS)) / FAN_TACH_PULSES_PER_REV;
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
    pinMode(extruderSrewPin, INPUT);

    pinMode (encoderPinA, INPUT);
    pinMode (encoderPinB, INPUT);
    pinMode(switchManualPin, INPUT);
    pinMode(switchLoadPin, INPUT); 
    pinMode(fanRSpeedPin, INPUT);
}

/// @brief Perform homing and no-load current calibration for the filament guide system.
/// @param calibrationTime_ms Duration in milliseconds for which to run the no-load current calibration. 
/// @param sampleInterval_ms Interval in milliseconds between current samples during calibration.
void HomingAndCalibration(int calibrationTime_ms, int sampleInterval_ms) {
    Serial.println("Starting no-load current calibration...");    
    bool calibrated = true;

    unsigned long calibStartTime = millis();
    unsigned long millisPrev = 0;

    float calib_spool_sum = 0.0;
    int calib_spool_samples = 0;
    analogWrite(motorSpoolPin, DAC_maxValue); // Run spool at full speed for calibration

    while (!calibrated) {
        unsigned long millisCurrent = millis();

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
void updateMeasurements() {
    float currentSpool = ina219_spool.getCurrent_mA();
    SpoolMotorCurrent += CurrentfilterAlpha*(currentSpool - SpoolMotorCurrent); // Simple low-pass filter to smooth current measurement for spool motor
    decaySpeed(); 
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

void initRaspberrySerial() {
    RaspberrySerial.begin(115200, SERIAL_8N1, raspberryPiRXPIn, raspberryPiTXPin);
    raspberrySerialIndex = 0;
    raspberrySerialReady = true;
    lastRaspberryCommandMs = millis();
}

void pollRaspberrySerial() {
    if (!raspberrySerialReady) {
        return;
    }

    while (RaspberrySerial.available() > 0) {
        char c = (char)RaspberrySerial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            raspberrySerialBuffer[raspberrySerialIndex] = '\0';
            if (raspberrySerialIndex > 0) {
                processRaspberryJsonLine(raspberrySerialBuffer);
            }
            raspberrySerialIndex = 0;
            continue;
        }

        if (raspberrySerialIndex < sizeof(raspberrySerialBuffer) - 1) {
            raspberrySerialBuffer[raspberrySerialIndex++] = c;
        } else {
            raspberrySerialIndex = 0;
        }
    }
}

void processRaspberryJsonLine(const char* line) {
    if (parseRaspberryCommands(line)) {
        applyRaspberryCommands();
        lastRaspberryCommandMs = millis();
    }
}

bool parseRaspberryCommands(const char* line) {
    // Robust framing: use the LAST '{' so any partial frame prepended by a
    // crashed/restarted sender is skipped, leaving only the complete object.
    const char* jsonStart = strrchr(line, '{');
    const char* jsonEnd = strrchr(line, '}');
    if (jsonStart == nullptr || jsonEnd == nullptr || jsonEnd < jsonStart) {
        return false;
    }

    size_t jsonLen = (size_t)(jsonEnd - jsonStart + 1);
    if (jsonLen == 0 || jsonLen >= 512) {
        return false;
    }

    char jsonPayload[512];
    memcpy(jsonPayload, jsonStart, jsonLen);
    jsonPayload[jsonLen] = '\0';

    StaticJsonDocument<768> doc;
    DeserializationError error = deserializeJson(doc, jsonPayload);
    if (error) {
        Serial.print("WARN: Raspberry JSON parse failed: ");
        Serial.println(error.c_str());
        return false;
    }

    raspberryCommands.pid_p_pulley_dia = doc["p1"] | doc["pid_p_pulley_dia"] | (doc["pid_p_pulley"] | raspberryCommands.pid_p_pulley_dia);
    raspberryCommands.pid_i_pulley_dia = doc["i1"] | doc["pid_i_pulley_dia"] | (doc["pid_i_pulley"] | raspberryCommands.pid_i_pulley_dia);
    raspberryCommands.pid_d_pulley_dia = doc["d1"] | doc["pid_d_pulley_dia"] | (doc["pid_d_pulley"] | raspberryCommands.pid_d_pulley_dia);
    raspberryCommands.pid_p_pulley_spd = doc["p2"] | doc["pid_p_pulley_spd"] | (doc["pid_p_pulley"] | raspberryCommands.pid_p_pulley_spd);
    raspberryCommands.pid_i_pulley_spd = doc["i2"] | doc["pid_i_pulley_spd"] | (doc["pid_i_pulley"] | raspberryCommands.pid_i_pulley_spd);
    raspberryCommands.pid_d_pulley_spd = doc["d2"] | doc["pid_d_pulley_spd"] | (doc["pid_d_pulley"] | raspberryCommands.pid_d_pulley_spd);
    raspberryCommands.pid_p_spool = doc["p3"] | doc["pid_p_spool"] | raspberryCommands.pid_p_spool;
    raspberryCommands.pid_i_spool = doc["i3"] | doc["pid_i_spool"] | raspberryCommands.pid_i_spool;
    raspberryCommands.pid_d_spool = doc["d3"] | doc["pid_d_spool"] | raspberryCommands.pid_d_spool;
    raspberryCommands.fan_speed_pct = doc["f"] | doc["fan_speed_pct"] | raspberryCommands.fan_speed_pct;
    raspberryCommands.spool_current_target_ma = doc["c"] | doc["spool_current_target_ma"] | raspberryCommands.spool_current_target_ma;
    raspberryCommands.target_diameter_mm = doc["td"] | doc["target_diameter_mm"] | raspberryCommands.target_diameter_mm;
    raspberryCommands.measured_diameter_mm = doc["md"] | doc["measured_diameter_mm"] | raspberryCommands.measured_diameter_mm;
    raspberryCommands.target_speed_mps = doc["ts"] | doc["target_speed_mps"] | raspberryCommands.target_speed_mps;

    const char* mode = doc["m"] | doc["target_mode"] | raspberryCommands.target_mode;
    if (strcmp(mode, "Dia") == 0 || strcmp(mode, "Spd") == 0) {
        strncpy(raspberryCommands.target_mode, mode, sizeof(raspberryCommands.target_mode) - 1);
        raspberryCommands.target_mode[sizeof(raspberryCommands.target_mode) - 1] = '\0';
    }

    return true;
}

void applyRaspberryCommands() {
    targetSpeed = constrain(raspberryCommands.target_speed_mps, 0.0f, 0.03f);
    setFanDutyPercent((uint8_t)constrain((int)raspberryCommands.fan_speed_pct, 0, 100));
    SetTorqueCurrent = constrain(raspberryCommands.spool_current_target_ma, 0.0f, 995.0f);

    RollerDiameterPID.setTunings(
        raspberryCommands.pid_p_pulley_dia,
        raspberryCommands.pid_i_pulley_dia,
        raspberryCommands.pid_d_pulley_dia
    );
    RollerPID.setTunings(
        raspberryCommands.pid_p_pulley_spd,
        raspberryCommands.pid_i_pulley_spd,
        raspberryCommands.pid_d_pulley_spd
    );
    SpoolPID.setTunings(
        raspberryCommands.pid_p_spool,
        raspberryCommands.pid_i_spool,
        raspberryCommands.pid_d_spool
    );
}

void sendTelemetryToRaspberry() {
    if (!raspberrySerialReady) {
        return;
    }

    TelemetryFromEsp32 telemetry = {
        (loadState != LOAD_IDLE),
        waitingForLoad,
        (float)speed,
        (float)fanRpm,
        (float)(traveledDistance * 1000.0),
        (float)SpoolMotorCurrent
    };

    StaticJsonDocument<320> doc;
    doc["load_mode"] = telemetry.load_mode;
    doc["waiting_for_load"] = telemetry.waiting_for_load;
    doc["filament_speed_mps"] = telemetry.filament_speed_mps;
    doc["fan_rpm"] = telemetry.fan_rpm;
    doc["diameter_travelled_mm"] = telemetry.diameter_travelled_mm;
    doc["spool_current_ma"] = telemetry.spool_current_ma;

    serializeJson(doc, RaspberrySerial);
    RaspberrySerial.print('\n');
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
