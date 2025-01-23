#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 6, 5, 8);
Encoder encoder = Encoder(2, 3, 512);

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

Encoder pendulum = Encoder(10, 11, 512);
void doPA() { pendulum.handleA(); }
void doPB() { pendulum.handleB(); }

PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);


float Kp = 6.0, Ki = 0.5, Kd = 1.5;
float setpoint = 0.0, input = 0.0, output = 0.0;
float lastError = 0.0, integral = 0.0;
unsigned long lastTime = 0;
float dt = 0.01;

void setup() {

  Serial.begin(115200);
  encoder.init();
  encoder.enableInterrupts(doA,doB);

  pendulum.init();
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);

  motor.controller = MotionControlType::torque;
  motor.linkSensor(&encoder);
  
  // driver
  driver.voltage_power_supply = 20; 
  driver.init();
  motor.linkDriver(&driver);
  motor.init();
  motor.initFOC();
}

void loop() {
  motor.loopFOC();
  pendulum.update();
  

  float input = constrainAngle(pendulum.getAngle() + M_PI);
  
  float error = setpoint - input;
  
  // om te checken of we al dicht bij genoeg zijn om te stabiliseren 
  if (abs(error) < 0.01) {
    integral = 0;
  } else {
    integral += error * dt;
  }
  

  integral = constrain(integral, -10, 10);
  //PID Control code hier 
  float derivative = (error - lastError) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  float volt_Lim = motor.voltage_limit * 0.7;
  output = constrain(output, -volt_Lim, volt_Lim);

  motor.move(output);
  delay((int)(dt * 1000));
  
  Serial.print("Angle: ");
  Serial.print(input);
  Serial.print(" Voltage: ");
  Serial.println(output);
}

float constrainAngle(float x) {
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0) {
    x += 2 * M_PI;
  }
  return x - M_PI;
}