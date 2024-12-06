#include <SimpleFOC.h>

//de 

MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 7);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 9, 5, 8);

float Kp = 1.0, Ki = 0.5, Kd = 1.5;
float setpoint = 0.0, input = 0.0, output = 0.0;
float lastError = 0.0, integral = 0.0;
unsigned long lastTime = 0;
float dt = 0.01;

void setup() {
  Serial.begin(9600);
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.init();
  motor.initFOC();
}

void loop() {
  motor.loopFOC();
  input = sensor.getAngle();
  float error = setpoint - input;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  motor.move(constrain(output, -12.0, 12.0));
  delay((int)(dt * 1000));
  Serial.print("Angle: ");
  Serial.print(input);
  Serial.print(" Voltage: ");
  Serial.println(output);
}