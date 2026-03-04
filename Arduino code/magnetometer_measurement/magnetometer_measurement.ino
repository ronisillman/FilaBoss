#include <Wire.h>
#include <Adafruit_AS5600.h>

Adafruit_AS5600 as5600 = Adafruit_AS5600();

float angle = 0.0;
float angleDegrees = 0.0;

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    Wire.begin();
    delay(100);
    
    // Initialize AS5600
    if (!as5600.begin()) {
        Serial.println("AS5600 not found!");
        while (1) delay(100);
    }
    
    Serial.println("AS5600 initialized successfully");
    Serial.println("Angle(°), Raw(0-4095), AGC, Magnet");
}

void loop() {
    // Read angle (0-4095 raw counts)
    angle = as5600.getAngle();
    
    // Convert to degrees: full rotation = 360°
    angleDegrees = (angle / 4095.0) * 360.0;
    
    // Read diagnostic info
    uint8_t agc = as5600.getAGC();
    bool magnetDetected = as5600.isMagnetDetected();
    
    // Print to serial plotter
    Serial.print(angleDegrees, 2);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.print(agc);
    Serial.print(",");
    Serial.println(magnetDetected ? 1 : 0);
    
    delay(500);  // 20 Hz update
}

// Optional: Diagnostic function
void printStatus() {
    Serial.println("\n--- AS5600 Status ---");
    Serial.print("Angle: ");
    Serial.println(as5600.getAngle());
    Serial.print("AGC: ");
    Serial.println(as5600.getAGC());
    Serial.print("Magnet Detected: ");
    Serial.println(as5600.isMagnetDetected() ? "Yes" : "No");
    Serial.println();
}