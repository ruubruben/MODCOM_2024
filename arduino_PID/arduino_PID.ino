#include <SimpleFOC.h>

// AS5048 Sensor - Change CS pin if necessary
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 8);

// Motor setup
BLDCMotor motor = BLDCMotor(11);  // Adjust pole pairs as needed
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);  // PWM pins for the SimpleFOC shield (adjust if necessary)

// Custom PID variables
float kp = 0.2;  // Proportional gain
float ki = 0.1;  // Integral gain
float kd = 0.08; // Derivative gain

float targetVelocity = 2.0;  // Target velocity in rad/s
float lastError = 0.0;       // For calculating derivative
float integral = 0.0;        // Integral term

void setup() {
  Serial.begin(115200);

  // Initialize sensor
  sensor.init();
  Serial.println("Sensor initialized.");

  // Link sensor to motor
  motor.linkSensor(&sensor);

  // Initialize motor driver
  driver.voltage_power_supply = 10;
  driver.init();
  motor.linkDriver(&driver);

  // Set the motor control mode to torque
  motor.controller = MotionControlType::torque;

  // Initialize motor and FOC
  motor.init();
  motor.initFOC();

  Serial.println("Motor and sensor linked. FOC initialized. Starting loop...");
}

void loop() {
  // Get the current velocity from the sensor
  float currentVelocity = sensor.getVelocity();

  // Custom PID control logic
  float error = targetVelocity - currentVelocity;
  integral += error * 0.1; // Assume a fixed loop time of 0.1s for simplicity
  float derivative = (error - lastError) / 0.1;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;

  // Clamp output to voltage limit
  if (output > motor.voltage_limit) output = motor.voltage_limit;
  if (output < -motor.voltage_limit) output = -motor.voltage_limit;

  // Set motor torque directly using the custom PID output
  motor.move(output);

  // Run the FOC algorithm to keep the motor aligned
  motor.loopFOC();

  // Print debug info
  Serial.print("Current Velocity: ");
  Serial.print(currentVelocity);
  Serial.print(" | Target Velocity: ");
  Serial.print(targetVelocity);
  Serial.print(" | PID Output: ");
  Serial.println(output);

  delay(100); // Loop delay to simulate control period
}