#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>


// BLDC motor init
BLDCMotor motor = BLDCMotor(11);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 6, 5, 8);
//Motor encoder init
Encoder encoder = Encoder(2, 3, 512);
// interrupt routine 
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


// pendulum encoder init
Encoder pendulum = Encoder(10, 11, 512);
// interrupt routine 
void doPA() { pendulum.handleA(); }
void doPB() { pendulum.handleB(); }
// PCI manager interrupt
PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);


float Kp = 1.0, Ki = 0.5, Kd = 1.5;
float setpoint = 0.0, input = 0.0, output = 0.0;
float lastError = 0.0, integral = 0.0;
unsigned long lastTime = 0;
float dt = 0.01;

void setup() {

  Serial.begin(115200);

  // initialise motor encoder hardware
  encoder.init();
  encoder.enableInterrupts(doA,doB);
  
  // init the pendulum encoder
  pendulum.init();
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);
  
  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // link the motor to the encoder
  motor.linkSensor(&encoder);
  
  // driver
  driver.voltage_power_supply = 20; 
  driver.init();
  // link the driver and the motor
  motor.linkDriver(&driver);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  motor.loopFOC();
  pendulum.update();
  
  // calculate the pendulum angle 
  float input = constrainAngle(pendulum.getAngle() + M_PI);
  
  float error = setpoint - input;
  
  // Anti-windup: Reset integral term if error is small
  if (abs(error) < 0.01) {
    integral = 0;
  } else {
    integral += error * dt;
  }
  
  // Clamp the integral term
  integral = constrain(integral, -10, 10);
  
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