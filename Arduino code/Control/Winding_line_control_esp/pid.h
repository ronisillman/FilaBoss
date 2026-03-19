// PID Controller Class for Arduino

class PID {
private:
  float kp, ki, kd;
  float setpoint;
  float integral;
  float previousError;
  unsigned long lastTime;
  float outputMin, outputMax, output;

public:
  // Constructor
  PID(float kp_val = 0.0, float ki_val = 0.0, float kd_val = 0.0) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    setpoint = 0.0;
    integral = 0.0;
    previousError = 0.0;
    lastTime = millis();
    outputMin = -255.0;
    outputMax = 255.0;
    output = 0.0;
  }

  // Set PID tuning parameters
  void setTunings(float kp_val, float ki_val, float kd_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
  }

  // Set the desired setpoint
  void setSetpoint(float target) {
    setpoint = target;
  }

  // Set output limits
  void setOutputLimits(float minOut, float maxOut) {
    outputMin = minOut;
    outputMax = maxOut;
  }

  // Compute PID output based on current feedback
  float compute(float feedback) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

    // Avoid division by zero - return previous output if no time has passed
    if (deltaTime == 0) return output;
    
    lastTime = currentTime;

    // Calculate error
    float error = setpoint - feedback;

    // Proportional term
    float proportional = kp * error;

    // Integral term with anti-windup
    integral += ki * error * deltaTime;
    if (integral > outputMax) integral = outputMax;
    if (integral < outputMin) integral = outputMin;

    // Derivative term
    float derivative = 0;
    if (deltaTime > 0) {
      derivative = kd * (error - previousError) / deltaTime;
    }

    // Store error for next iteration
    previousError = error;

    // Calculate output
    output = proportional + integral + derivative;

    // Limit output
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;

    return output;
  }

  // Reset the controller
  void reset() {
    integral = 0.0;
    previousError = 0.0;
    lastTime = millis();
  }

  // Get current setpoint
  float getSetpoint() {
    return setpoint;
  }

  // Get current integral value
  float getIntegral() {
    return integral;
  }
  float getOutput() {
    return output;
  }
};
