#include <ArduinoJson.h>

#define RXD2 25
#define TXD2 26

static constexpr unsigned long BAUD_RATE = 115200;
static constexpr unsigned long TELEMETRY_PERIOD_MS = 500;

HardwareSerial RaspberrySerial(2);

struct TelemetryFromEsp32 {
    bool load_mode;
    float filament_speed_mps;
    float fan_rpm;
    float diameter_travelled_mm;
};

struct CommandsToEsp32 {
    float pid_p_pulley;
    float pid_i_pulley;
    float pid_d_pulley;
    float pid_p_spool;
    float pid_i_spool;
    float pid_d_spool;
    uint8_t fan_speed_pct;
    char target_mode[4];
    float target_diameter_mm;
    float target_speed_mps;
};

CommandsToEsp32 raspberryCommands = {
    5000.0f,
    1500.0f,
    5.0f,
    7000.0f,
    2500.0f,
    5.0f,
    100,
    "Spd",
    1.75f,
    0.01f,
};

TelemetryFromEsp32 telemetry = {
    false,
    0.0f,
    0.0f,
    0.0f,
};

char raspberrySerialBuffer[256];
uint16_t raspberrySerialIndex = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastSimulationMs = 0;
float simulatedSpeedMps = 0.0f;
float simulatedDistanceMm = 0.0f;

void initRaspberrySerial();
void pollRaspberrySerial();
void processRaspberryJsonLine(const char* line);
bool parseRaspberryCommands(const char* line);
void applyRaspberryCommands();
void sendTelemetryToRaspberry();
void updateSimulatedTelemetry();

void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("Standalone Raspberry serial JSON test ready.");
    initRaspberrySerial();
}

void loop() {
    pollRaspberrySerial();
    updateSimulatedTelemetry();

    unsigned long now = millis();
    if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = now;
        sendTelemetryToRaspberry();
    }
}

void initRaspberrySerial() {
    RaspberrySerial.begin(BAUD_RATE, SERIAL_8N1, RXD2, TXD2);
    raspberrySerialIndex = 0;
    lastTelemetryMs = millis();
    lastSimulationMs = millis();
    Serial.println("Raspberry UART2 initialized on GPIO25/GPIO26.");
}

void pollRaspberrySerial() {
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
    }
}

bool parseRaspberryCommands(const char* line) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, line);
    if (error) {
        Serial.print("WARN: Raspberry JSON parse failed: ");
        Serial.println(error.c_str());
        return false;
    }

    raspberryCommands.pid_p_pulley = doc["pid_p_pulley"] | raspberryCommands.pid_p_pulley;
    raspberryCommands.pid_i_pulley = doc["pid_i_pulley"] | raspberryCommands.pid_i_pulley;
    raspberryCommands.pid_d_pulley = doc["pid_d_pulley"] | raspberryCommands.pid_d_pulley;
    raspberryCommands.pid_p_spool = doc["pid_p_spool"] | raspberryCommands.pid_p_spool;
    raspberryCommands.pid_i_spool = doc["pid_i_spool"] | raspberryCommands.pid_i_spool;
    raspberryCommands.pid_d_spool = doc["pid_d_spool"] | raspberryCommands.pid_d_spool;
    raspberryCommands.fan_speed_pct = doc["fan_speed_pct"] | raspberryCommands.fan_speed_pct;
    raspberryCommands.target_diameter_mm = doc["target_diameter_mm"] | raspberryCommands.target_diameter_mm;
    raspberryCommands.target_speed_mps = doc["target_speed_mps"] | raspberryCommands.target_speed_mps;

    const char* mode = doc["target_mode"] | raspberryCommands.target_mode;
    if (strcmp(mode, "Dia") == 0 || strcmp(mode, "Spd") == 0) {
        strncpy(raspberryCommands.target_mode, mode, sizeof(raspberryCommands.target_mode) - 1);
        raspberryCommands.target_mode[sizeof(raspberryCommands.target_mode) - 1] = '\0';
    }

    return true;
}

void applyRaspberryCommands() {
    Serial.printf(
        "RX pidP=%.2f pidI=%.2f pidD=%.2f fan=%u mode=%s targetDia=%.3f targetSpeed=%.4f\n",
        raspberryCommands.pid_p_pulley,
        raspberryCommands.pid_i_pulley,
        raspberryCommands.pid_d_pulley,
        (unsigned int)raspberryCommands.fan_speed_pct,
        raspberryCommands.target_mode,
        raspberryCommands.target_diameter_mm,
        raspberryCommands.target_speed_mps
    );
}

void updateSimulatedTelemetry() {
    unsigned long now = millis();
    float dt = (now - lastSimulationMs) / 1000.0f;
    lastSimulationMs = now;

    float targetSpeed = constrain(raspberryCommands.target_speed_mps, 0.0f, 0.03f);
    if (simulatedSpeedMps < targetSpeed) {
        simulatedSpeedMps = min(targetSpeed, simulatedSpeedMps + 0.002f);
    } else {
        simulatedSpeedMps = max(targetSpeed, simulatedSpeedMps - 0.002f);
    }

    simulatedDistanceMm += simulatedSpeedMps * dt * 1000.0f;
    if (simulatedDistanceMm < 0.0f) {
        simulatedDistanceMm = 0.0f;
    }

    telemetry.load_mode = (strcmp(raspberryCommands.target_mode, "Dia") == 0);
    telemetry.filament_speed_mps = simulatedSpeedMps;
    telemetry.fan_rpm = (float)constrain((int)raspberryCommands.fan_speed_pct, 0, 100) * 30.0f;
    telemetry.diameter_travelled_mm = simulatedDistanceMm;
}

void sendTelemetryToRaspberry() {
    StaticJsonDocument<256> doc;
    doc["load_mode"] = telemetry.load_mode;
    doc["filament_speed_mps"] = telemetry.filament_speed_mps;
    doc["fan_rpm"] = telemetry.fan_rpm;
    doc["diameter_travelled_mm"] = telemetry.diameter_travelled_mm;

    serializeJson(doc, RaspberrySerial);
    RaspberrySerial.print('\n');
}