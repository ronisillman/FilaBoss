// =============================================================================
// pulley_pid_autotune.ino
// Standalone Åström-Hägglund relay autotuner for the pulley speed PID.
//
// Uses the EXACT same PCNT encoder speed measurement as Winding_line_control_esp.
// After completing the relay test it prints Ziegler-Nichols Kp/Ki/Kd to Serial.
// Enter the printed values into the PID menu of the main UI (Pulley entry).
//
// HOW IT WORKS:
//   1. Warmup  – drives motor at RELAY_HIGH for WARMUP_MS so speed reaches
//                the vicinity of TARGET_SPEED_MPS before the test begins.
//   2. Relay   – output switches between RELAY_HIGH and RELAY_LOW every time
//                speed crosses TARGET_SPEED_MPS.  This forces a steady
//                oscillation whose period (Tu) and amplitude (a) reveal the
//                ultimate gain: Ku = 4*d / (π*a), where d = relay half-swing.
//   3. Results – after N_CYCLES complete oscillations the motor stops and
//                Ziegler–Nichols Kp/Ki/Kd (and a conservative PI alternative)
//                are printed over Serial.
//
// TUNING THE TEST PARAMETERS (at top of file):
//   TARGET_SPEED_MPS  – pick a representative winding speed (default 10 mm/s).
//   RELAY_HIGH        – raise this if speed never reaches the target during
//                       warmup; lower it if the motor runs too fast.
//                       Aim for the motor to oscillate ±30-50% around the target.
//   N_CYCLES          – more cycles = more accurate average; 6 is usually enough.
// =============================================================================

#include "driver/pulse_cnt.h"
#include "pid.h"
#include <math.h>

// ── Pin definitions (must match main board) ──────────────────────────────────
#define motorRollerPin  12
#define encoderPinA     27
#define encoderPinB     14

// ── Encoder / speed constants – copied verbatim from main sketch ─────────────
static const float         encoderResolution    = 600.0f;
static const float         encoderEdgesPerPulse = 2.0f;
static const float         rollerRadius         = 0.0119f;   // m
static const float         speedCal             = 500.0f / 715.23f;
static const float         SpeedFilterAlpha          = 0.01f;  // same as main code – used during PID verify
static const float         AUTOTUNE_SPEED_FILTER_ALPHA = 0.4f;  // faster response needed during relay test
static float               currentFilterAlpha         = SpeedFilterAlpha;
static const float         SPEED_DEADBAND_MPS   = 0.0002f;
static const unsigned long STOP_TIMEOUT_US       = 300000UL; // 300 ms
static const int           PCNT_H_LIM            = 32767;

// ── Autotune parameters ───────────────────────────────────────────────────────
// TARGET_SPEED_MPS: the speed setpoint the relay oscillates around.
static const float TARGET_SPEED_MPS = 0.010f;   // 10 mm/s

// RELAY_HIGH / RELAY_LOW: PWM output values toggled by the relay.
// The motor output limit in the main code is DAC_maxValue*0.6 ≈ 153.
// Start around 110 and adjust until the motor visibly oscillates around target.
static const int RELAY_HIGH = 110;   // PWM when speed < target  (0-153)
static const int RELAY_LOW  = 0;     // PWM when speed > target

// N_CYCLES: number of complete oscillation cycles to average over.
static const int N_CYCLES = 6;

// WARMUP_MS: time in ms to spin up before the relay test starts.
static const unsigned long WARMUP_MS = 4000UL;

// ── PCNT state ────────────────────────────────────────────────────────────────
static pcnt_unit_handle_t    encoderPcntUnit    = nullptr;
static pcnt_channel_handle_t encoderPcntChannel = nullptr;
static bool encoderPcntReady = false;
static int  prevPcntCount    = 0;

// ── Speed state (same variables/types as main sketch) ─────────────────────────
static double        speed             = 0.0;
static unsigned long encoderPrevTime   = 0;
static unsigned long lastEncoderPeriod = 100000UL;
static bool          hasValidPulse     = false;
static bool          encoderPulseEvent = false;

// ── Relay autotune state ──────────────────────────────────────────────────────
enum AutotuneState { AT_WARMUP, AT_RUNNING, AT_DONE, AT_PID_VERIFY };
static AutotuneState atState = AT_WARMUP;

static unsigned long warmupStartMs = 0;
static int           relayOutput   = RELAY_HIGH;
static bool          speedWasAbove = false;   // last side of setpoint seen
static bool          firstCrossing = true;    // ignore the very first crossing

// Accumulate statistics over half-cycles
static int   halfCycleCount = 0;
static float halfPeriodSum  = 0.0f;   // sum of all half-period durations (µs)
static float ampSum         = 0.0f;   // sum of half-amplitudes (m/s) per half-cycle

// Per-half-cycle extremes
static float segPeak   = 0.0f;
static float segTrough = 1e9f;

static unsigned long lastCrossUs = 0;

// ── PID verification state ────────────────────────────────────────────────────
// Gains stored after autotune completes; used for the verification run.
static float verifyKp = 0.0f;
static float verifyKi = 0.0f;
static float verifyKd = 0.0f;
static PID   verifyPID;
static unsigned long lastCtrlMs   = 0;
static unsigned long lastPrintMs  = 0;
static const unsigned long CTRL_PERIOD_MS  = 10UL;   // matches main code MOTOR_CTRL_PERIOD_MS
static const unsigned long PRINT_PERIOD_MS = 200UL;  // print speed 5 times/s
static const float OUTPUT_LIMIT_MAX = 153.0f;        // DAC_maxValue * 0.6, same as main code

// Progress print throttle
static unsigned long lastProgressMs = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
void initPCNT();
void pollPcntPulseEvent();
void updateSpeedByPulse();
void decaySpeed();
void printResults();
void startPidVerify(float kp, float ki, float kd);

// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("==============================================");
    Serial.println("   Pulley Speed PID – Relay Autotune");
    Serial.println("==============================================");
    Serial.printf("  Target speed : %.1f mm/s\n", TARGET_SPEED_MPS * 1000.0f);
    Serial.printf("  Relay HIGH   : %d PWM\n", RELAY_HIGH);
    Serial.printf("  Relay LOW    : %d PWM\n", RELAY_LOW);
    Serial.printf("  Cycles       : %d\n", N_CYCLES);
    Serial.println("----------------------------------------------");
    Serial.println("Ensure filament is threaded and motor is free.");
    Serial.println("Starting warmup...");

    pinMode(motorRollerPin, OUTPUT);
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

    delay(3000);

    initPCNT();

    delay(3000);

    // Capture initial encoder count to avoid spurious first delta
    if (encoderPcntReady) {
        int cnt = 0;
        pcnt_unit_get_count(encoderPcntUnit, &cnt);
        prevPcntCount = cnt;
    }

    encoderPrevTime   = micros();
    lastEncoderPeriod = 100000UL;

    if (!encoderPcntReady) {
        Serial.println("FATAL: PCNT encoder init failed. Check wiring on pins 27/14 and Serial output above for the error.");
        while (1) { delay(100); }
    }

    warmupStartMs = millis();
    atState       = AT_WARMUP;
    analogWrite(motorRollerPin, RELAY_HIGH);
}

// =============================================================================
void loop() {
    // ── Poll encoder & update speed (identical pattern to main sketch) ────────
    pollPcntPulseEvent();
    if (encoderPulseEvent) {
        encoderPulseEvent = false;
        updateSpeedByPulse();
    }
    decaySpeed();

    unsigned long nowMs = millis();

    // ── WARMUP phase ──────────────────────────────────────────────────────────
    if (atState == AT_WARMUP) {
        // Progress report during warmup – also prints raw PCNT count so you
        // can confirm the encoder is being counted even before speed filters up.
        if (nowMs - lastProgressMs >= 1000UL) {
            int rawCount = 0;
            pcnt_unit_get_count(encoderPcntUnit, &rawCount);
            Serial.printf("  Warmup: speed = %.4f m/s  (target %.4f m/s)  PCNT_raw = %d\n",
                          (float)speed, TARGET_SPEED_MPS, rawCount);
            lastProgressMs = nowMs;
        }

        if (nowMs - warmupStartMs >= WARMUP_MS) {
            Serial.println("Warmup done. Starting relay test.");
            atState = AT_RUNNING;
            currentFilterAlpha = AUTOTUNE_SPEED_FILTER_ALPHA; // fast filter for relay test

            // Initialise relay direction based on current speed
            speedWasAbove = ((float)speed >= TARGET_SPEED_MPS);
            relayOutput   = speedWasAbove ? RELAY_LOW : RELAY_HIGH;
            analogWrite(motorRollerPin, relayOutput);

            firstCrossing = true;
            segPeak       = (float)speed;
            segTrough     = (float)speed;
            lastCrossUs   = micros();
            lastProgressMs = nowMs;
        }
        return;
    }

    // ── DONE phase ────────────────────────────────────────────────────────────
    if (atState == AT_DONE) {
        return; // results printed; startPidVerify() transitions to AT_PID_VERIFY
    }

    // ── PID VERIFY phase ──────────────────────────────────────────────────────
    if (atState == AT_PID_VERIFY) {
        // Stop verification if any character sent on Serial.
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            analogWrite(motorRollerPin, 0);
            Serial.println("PID verify stopped by user.");
            atState = AT_DONE;
            return;
        }

        // Motor control at CTRL_PERIOD_MS – matches main code MOTOR_CTRL_PERIOD_MS path.
        if (nowMs - lastCtrlMs >= CTRL_PERIOD_MS) {
            verifyPID.setSetpoint(TARGET_SPEED_MPS);
            float pidOut = verifyPID.compute((float)speed);
            analogWrite(motorRollerPin, (int)roundf(pidOut));
            lastCtrlMs = nowMs;
        }

        // Print speed periodically.
        if (nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
            Serial.printf("  PID verify: speed = %6.4f m/s (%5.2f mm/s) | target = %.4f m/s | output = %3d\n",
                          (float)speed,
                          (float)speed * 1000.0f,
                          TARGET_SPEED_MPS,
                          (int)roundf(verifyPID.getOutput()));
            lastPrintMs = nowMs;
        }
        return;
    }

    // ── RUNNING phase ─────────────────────────────────────────────────────────

    // Track extremes within the current half-cycle
    float spd = (float)speed;
    if (spd > segPeak)   segPeak   = spd;
    if (spd < segTrough) segTrough = spd;

    // Detect zero-crossing of (speed - TARGET_SPEED_MPS)
    bool aboveNow = (spd >= TARGET_SPEED_MPS);
    if (aboveNow != speedWasAbove) {
        unsigned long nowUs = micros();

        if (!firstCrossing) {
            // Record this half-cycle's statistics
            unsigned long halfPeriodUs = nowUs - lastCrossUs;
            halfPeriodSum += (float)halfPeriodUs;
            float halfAmp = (segPeak - segTrough) * 0.5f;
            ampSum        += halfAmp;
            halfCycleCount++;

            Serial.printf("  Half-cycle #%2d | period = %6.1f ms | half-amp = %.4f m/s\n",
                          halfCycleCount,
                          (float)halfPeriodUs / 1000.0f,
                          halfAmp);

            // Check if we have collected enough complete cycles (2 half-cycles = 1 full cycle)
            if (halfCycleCount >= N_CYCLES * 2) {
                atState = AT_DONE;
                analogWrite(motorRollerPin, 0);
                printResults();
                return;
            }
        } else {
            firstCrossing = false;
            Serial.println("  First crossing detected – collecting data...");
        }

        // Toggle relay output
        relayOutput   = aboveNow ? RELAY_LOW : RELAY_HIGH;
        analogWrite(motorRollerPin, relayOutput);
        speedWasAbove = aboveNow;
        lastCrossUs   = nowUs;
        segPeak       = spd;
        segTrough     = spd;
    } else {
        speedWasAbove = aboveNow;
    }

    // Periodic progress print
    if (nowMs - lastProgressMs >= 2000UL) {
        Serial.printf("  Running: speed = %.4f m/s | output = %3d | cycles = %d/%d\n",
                      spd, relayOutput, halfCycleCount / 2, N_CYCLES);
        lastProgressMs = nowMs;
    }
}

// =============================================================================
void printResults() {
    // Tu = average full period (2 × average half-period), in seconds
    float avgHalfPeriodUs = halfPeriodSum / (float)halfCycleCount;
    float Tu = (avgHalfPeriodUs * 2.0f) / 1e6f;

    // a = average oscillation half-amplitude (m/s)
    float a = ampSum / (float)halfCycleCount;

    // Relay half-amplitude in PWM counts
    float d = (float)(RELAY_HIGH - RELAY_LOW) * 0.5f;

    // Ultimate gain (PWM / (m/s)) – matches units used by PulleyPID in main code
    float Ku = (4.0f * d) / ((float)M_PI * a);

    // ── Ziegler-Nichols PID ───────────────────────────────────────────────────
    float Kp_zn = 0.6f  * Ku;
    float Ti_zn = 0.5f  * Tu;
    float Td_zn = 0.125f * Tu;
    float Ki_zn = Kp_zn / Ti_zn;
    float Kd_zn = Kp_zn * Td_zn;

    // ── Conservative Z-N PI (less overshoot, good starting point) ────────────
    float Kp_pi = 0.45f * Ku;
    float Ti_pi = 0.833f * Tu;
    float Ki_pi = Kp_pi / Ti_pi;

    Serial.println("\n==============================================");
    Serial.println("         AUTOTUNE RESULTS");
    Serial.println("==============================================");
    Serial.printf("  Ultimate gain  Ku = %.2f  (PWM / (m/s))\n", Ku);
    Serial.printf("  Ultimate period Tu = %.3f s\n", Tu);
    Serial.printf("  Oscillation amp a  = %.4f m/s (%.2f mm/s)\n", a, a * 1000.0f);
    Serial.println("----------------------------------------------");
    Serial.println("  Ziegler-Nichols PID (aggressive, may overshoot):");
    Serial.printf("    Kp = %8.4f\n", Kp_zn);
    Serial.printf("    Ki = %8.4f\n", Ki_zn);
    Serial.printf("    Kd = %8.4f\n", Kd_zn);
    Serial.println("----------------------------------------------");
    Serial.println("  Conservative PI (recommended starting point):");
    Serial.printf("    Kp = %8.4f\n", Kp_pi);
    Serial.printf("    Ki = %8.4f\n", Ki_pi);
    Serial.printf("    Kd =   0.0000\n");
    Serial.println("==============================================");
    Serial.println("Motor stopped. Enter these values in the");
    Serial.println("PID menu under 'Pulley' in the main UI.");
    Serial.println("==============================================");

    // Immediately start PID verification with Ziegler-Nichols PID gains.
    startPidVerify(Kp_zn, Ki_zn, Kd_zn);
}

void startPidVerify(float kp, float ki, float kd) {
    verifyKp = kp;
    verifyKi = ki;
    verifyKd = kd;

    verifyPID.setTunings(kp, ki, kd);
    verifyPID.setOutputLimits(0.0f, OUTPUT_LIMIT_MAX);
    verifyPID.setSetpoint(TARGET_SPEED_MPS);
    verifyPID.reset();

    Serial.println("\n==============================================");
    Serial.println("   PID VERIFICATION - Ziegler-Nichols PID gains");
    Serial.printf("   Kp=%.4f  Ki=%.4f  Kd=%.4f\n", kp, ki, kd);
    Serial.printf("   Target: %.1f mm/s\n", TARGET_SPEED_MPS * 1000.0f);
    Serial.println("   Send any key on Serial to stop.");
    Serial.println("==============================================");

    lastCtrlMs  = millis();
    lastPrintMs = millis();
    atState     = AT_PID_VERIFY;
    currentFilterAlpha = SpeedFilterAlpha; // restore main-code filter for verify
}

// =============================================================================
// PCNT & speed functions – copied verbatim from Winding_line_control_esp.ino
// =============================================================================

void initPCNT() {
    encoderPcntReady = false;

    pcnt_unit_config_t encoderUnitConfig = {
        .low_limit  = -PCNT_H_LIM,
        .high_limit =  PCNT_H_LIM,
    };
    esp_err_t err = pcnt_new_unit(&encoderUnitConfig, &encoderPcntUnit);
    if (err != ESP_OK) {
        Serial.printf("ERROR: encoder pcnt_new_unit failed (%d)\n", (int)err);
        return;
    }

    pcnt_chan_config_t encoderChanConfig = {
        .edge_gpio_num  = encoderPinA,
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
    Serial.println("PCNT encoder ready.");
}

void pollPcntPulseEvent() {
    if (!encoderPcntReady) return;

    int pcntCount = 0;
    if (pcnt_unit_get_count(encoderPcntUnit, &pcntCount) != ESP_OK) return;

    int countDelta = pcntCount - prevPcntCount;
    if (countDelta != 0 && abs(countDelta) < 1000) {
        encoderPulseEvent = true;
        prevPcntCount     = pcntCount;
    }
}

void updateSpeedByPulse() {
    unsigned long currentTime = micros();
    unsigned long period      = currentTime - encoderPrevTime;
    if (period == 0) return;

    float revPerSec = 1000000.0f / ((float)period * encoderResolution * encoderEdgesPerPulse);
    float rawSpeed  = revPerSec * (2.0f * PI * rollerRadius);
    rawSpeed *= speedCal;
    speed += currentFilterAlpha * (rawSpeed - speed);

    lastEncoderPeriod = period;
    encoderPrevTime   = currentTime;
    hasValidPulse     = true;
}

void decaySpeed() {
    unsigned long currentTime = micros();
    unsigned long gap_period  = currentTime - encoderPrevTime;

    if (!hasValidPulse) {
        speed = 0.0;
        return;
    }
    if (gap_period > STOP_TIMEOUT_US) {
        speed = 0.0;
        return;
    }
    if (gap_period > lastEncoderPeriod) {
        float revPerSec = 1000000.0 / ((float)gap_period * encoderResolution * encoderEdgesPerPulse);
        float rawSpeed  = revPerSec * (2.0 * PI * rollerRadius);
        rawSpeed *= speedCal;
        speed += currentFilterAlpha * (rawSpeed - speed);
    }
    if (fabs(speed) < SPEED_DEADBAND_MPS) {
        speed = 0.0;
    }
}
