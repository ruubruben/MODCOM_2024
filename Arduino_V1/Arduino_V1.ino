#include <SimpleFOC.h>

// AS5048 Sensor - Change CS pin if necessary
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 7);

// Motor setup
BLDCMotor motor = BLDCMotor(14);  // Adjust pole pairs as needed
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 9, 5, 8);  // PWM pins for the SimpleFOC shield (adjust if necessary)

// Commander interface for serial control (optional)
Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
.  c
  // Initialize sensor
  sensor.init();
  Serial.println("Sensor initialized.");

  // Link sensor to motor
  motor.linkSensor(&sensor);

  // Initialize motor driver
  driver.voltage_power_supply = 10;
  driver.init();
  motor.linkDriver(&driver);

  // Set the motor control mode
  motor.controller = MotionControlType::torque;

  // PID controller settings for torque control
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 1;
  motor.voltage_limit = 10;

  // Low-pass filter settings for velocity
  motor.LPF_velocity.Tf = 0.01;

  // Angle loop controller and velocity limit
  motor.P_angle.P = 20;
  motor.velocity_limit = 50;

  // Initialize motor and FOC
  motor.init();
  motor.initFOC();

  // Set initial target value for torque control
  motor.target = 2;

  // Commander setup for serial control (optional)
  command.add('A', onMotor, "motor");

  Serial.println("Motor and sensor linked. FOC initialized. Starting loop...");
}

void loop() {
  // Run the FOC algorithm to keep the motor in sync with the sensor
  motor.loopFOC();

  // Move the motor based on the current target (torque control in this case)
  motor.move(20);

  // Print the current sensor angle to Serial Monitor for debugging
  Serial.print("Current sensor angle: ");
  Serial.println(sensor.getAngle());

  // User command processing
  command.run();

  // Optional small delay to control Serial output rate
  delay(100);
}