#include <FastAccelStepper.h>
#include <TMCStepper.h>

// Minimal standalone guide movement test (5 cm) using FastAccelStepper.

// ESP32 pins used in your current control sketch
const int STEP_PIN = 2;
const int DIR_PIN = 15;
const int LIMIT_SWITCH_PIN = 35;
const bool LIMIT_SWITCH_ACTIVE_HIGH = true;
const int TMC_UART_RX_PIN = 5;
const int TMC_UART_TX_PIN = 13;
const uint32_t TMC_UART_BAUD = 115200;

// Mechanics
const float LEADSCREW_PITCH_M_PER_REV = 0.004f; // 4 mm/rev
const float STEPS_PER_REV_FULL = 200.0f;
const uint16_t DEFAULT_MICROSTEPS = 16;
const float TMC_R_SENSE = 0.11f;
const uint16_t TMC_RUN_CURRENT_MA = 1000;

// Motion target
const float MOVE_DISTANCE_M = 0.05f; // 5 cm

// Motion limits (start conservative; increase after verifying direction/mechanics)
const float MAX_SPEED_STEPS_S = 10000.0f;
const float ACCEL_STEPS_S2 = 10000.0f;

// If motion goes opposite, flip this and re-upload.
const bool DIR_INVERTED = false;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* guideStepper = nullptr;
HardwareSerial TMCSerial(1);
TMC2209Stepper tmcDriver(&TMCSerial, TMC_R_SENSE, 0);

long targetSteps = 0;
bool safetyStopped = false;
bool tmcUartReady = false;
bool moveDonePrinted = true;
int8_t moveDirection = 1; // +1 forward, -1 reverse
uint16_t configuredMicrosteps = DEFAULT_MICROSTEPS;
uint32_t configuredMaxSpeed = (uint32_t)MAX_SPEED_STEPS_S;
uint32_t configuredAcceleration = (uint32_t)ACCEL_STEPS_S2;
char serialBuffer[32];
uint8_t serialIndex = 0;

long distanceToSteps(float distanceM);
void applyMotionProfile();

bool isValidMicrosteps(uint16_t value) {
  switch (value) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
      return true;
    default:
      return false;
  }
}

void applyMicrosteps(uint16_t value) {
  if (!isValidMicrosteps(value)) {
    Serial.println("Invalid microsteps. Use: 1,2,4,8,16,32,64,128,256");
    return;
  }

  configuredMicrosteps = value;
  if (tmcUartReady) {
    tmcDriver.microsteps(configuredMicrosteps);
  }

  Serial.print("Microsteps set to: ");
  Serial.println(configuredMicrosteps);
}

void applyMotionProfile() {
  if (guideStepper == nullptr) {
    return;
  }

  guideStepper->setSpeedInHz(configuredMaxSpeed);
  guideStepper->setAcceleration(configuredAcceleration);
}

void startMove5cm() {
  if (guideStepper == nullptr) {
    return;
  }

  if (safetyStopped) {
    Serial.println("Blocked by safety stop. Send RESET first.");
    return;
  }

  long deltaSteps = distanceToSteps(MOVE_DISTANCE_M);
  long currentPos = guideStepper->getCurrentPosition();
  targetSteps = currentPos + (moveDirection > 0 ? deltaSteps : -deltaSteps);
  guideStepper->moveTo(targetSteps);
  moveDonePrinted = false;

  Serial.print("Move command accepted. Target steps: ");
  Serial.println(targetSteps);
}

void parseSerialCommand(char* cmd) {
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "HELP") == 0) {
    Serial.println("Commands: M=8, D=+ or D=-, V=1000, A=10000, GO, STOP, RESET");
    return;
  }

  if (cmd[0] == 'M' || cmd[0] == 'm') {
    char* valuePtr = cmd + 1;
    if (*valuePtr == '=') valuePtr++;
    uint16_t m = (uint16_t)atoi(valuePtr);
    applyMicrosteps(m);
    return;
  }

  if (cmd[0] == 'D' || cmd[0] == 'd') {
    char* valuePtr = cmd + 1;
    if (*valuePtr == '=') valuePtr++;

    if (*valuePtr == '+' || *valuePtr == '1') {
      moveDirection = 1;
      Serial.println("Direction set to: +");
      return;
    }
    if (*valuePtr == '-' || *valuePtr == '0') {
      moveDirection = -1;
      Serial.println("Direction set to: -");
      return;
    }

    Serial.println("Invalid direction. Use D=+ or D=-");
    return;
  }

  if (cmd[0] == 'V' || cmd[0] == 'v') {
    char* valuePtr = cmd + 1;
    if (*valuePtr == '=') valuePtr++;
    uint32_t speed = (uint32_t)atol(valuePtr);
    if (speed == 0) {
      Serial.println("Invalid speed. Use V=<steps_per_second>, e.g. V=1000");
      return;
    }

    configuredMaxSpeed = speed;
    applyMotionProfile();
    Serial.print("Max speed set to (steps/s): ");
    Serial.println(configuredMaxSpeed);
    return;
  }

  if (cmd[0] == 'A' || cmd[0] == 'a') {
    char* valuePtr = cmd + 1;
    if (*valuePtr == '=') valuePtr++;
    uint32_t accel = (uint32_t)atol(valuePtr);
    if (accel == 0) {
      Serial.println("Invalid accel. Use A=<steps_per_second^2>, e.g. A=10000");
      return;
    }

    configuredAcceleration = accel;
    applyMotionProfile();
    Serial.print("Acceleration set to (steps/s^2): ");
    Serial.println(configuredAcceleration);
    return;
  }

  if (strcmp(cmd, "GO") == 0 || strcmp(cmd, "go") == 0) {
    startMove5cm();
    return;
  }

  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "stop") == 0) {
    if (guideStepper != nullptr) {
      guideStepper->forceStopAndNewPosition(guideStepper->getCurrentPosition());
      moveDonePrinted = true;
    }
    Serial.println("Motion stopped.");
    return;
  }

  if (strcmp(cmd, "RESET") == 0 || strcmp(cmd, "reset") == 0) {
    safetyStopped = false;
    Serial.println("Safety stop reset.");
    return;
  }

  Serial.println("Unknown command. Type help");
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialBuffer[serialIndex] = '\0';
      if (serialIndex > 0) {
        parseSerialCommand(serialBuffer);
      }
      serialIndex = 0;
      continue;
    }

    if (serialIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[serialIndex++] = c;
    } else {
      serialIndex = 0;
    }
  }
}

void initTmc2209Uart() {
  TMCSerial.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);
  delay(50);

  tmcDriver.begin();
  tmcDriver.pdn_disable(true);
  tmcDriver.mstep_reg_select(true);
  tmcDriver.I_scale_analog(false);
  tmcDriver.toff(5);
  tmcDriver.blank_time(24);
  tmcDriver.microsteps(configuredMicrosteps);
  tmcDriver.rms_current(TMC_RUN_CURRENT_MA);
  tmcDriver.pwm_autoscale(true);

  uint8_t conn = tmcDriver.test_connection();
  tmcUartReady = (conn == 0);

  Serial.print("TMC2209 UART test_connection() = ");
  Serial.println(conn);
}

long distanceToSteps(float distanceM) {
  const float stepsPerRev = STEPS_PER_REV_FULL * (float)configuredMicrosteps;
  const float stepsPerMeter = stepsPerRev / LEADSCREW_PITCH_M_PER_REV;
  return (long)(distanceM * stepsPerMeter + 0.5f);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT);

  initTmc2209Uart();

  engine.init();
  guideStepper = engine.stepperConnectToPin(STEP_PIN);
  if (guideStepper == nullptr) {
    Serial.println("ERROR: stepperConnectToPin failed.");
    return;
  }

  guideStepper->setDirectionPin(DIR_PIN, DIR_INVERTED);
  guideStepper->setAutoEnable(false);
  applyMotionProfile();
  guideStepper->setCurrentPosition(0);

  if (!tmcUartReady) {
    Serial.println("WARN: TMC2209 UART not responding; microstep/current settings may be ignored.");
  }

  Serial.println("Guide 5 cm move test ready");
  Serial.println("Commands: M=8, D=+ or D=-, V=1000, A=10000, GO, STOP, RESET");
}

void loop() {
  processSerialInput();

  if (guideStepper == nullptr) {
    return;
  }

  bool limitHit = (digitalRead(LIMIT_SWITCH_PIN) == (LIMIT_SWITCH_ACTIVE_HIGH ? HIGH : LOW));
  if (limitHit && !safetyStopped) {
    safetyStopped = true;
    guideStepper->forceStopAndNewPosition(guideStepper->getCurrentPosition());
    Serial.println("SAFETY STOP: limit switch hit.");
    return;
  }

  if (safetyStopped) {
    return;
  }

  if (!guideStepper->isRunning() && !moveDonePrinted) {
    moveDonePrinted = true;
    Serial.println("Move complete.");
  }
}
