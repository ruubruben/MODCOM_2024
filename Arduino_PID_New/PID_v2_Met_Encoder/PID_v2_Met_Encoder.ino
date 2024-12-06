#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

MagneticSensorSPI motorSensor = MagneticSensorSPI(AS5048_SPI, 7);
BLDCMotor motor = BLDCMotor(14);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// pendulum encoder init
Encoder pendulum = Encoder(A0,A1, 1024);
// interrupt routine 
void doPA(){pendulum.handleA();}
void doPB(){pendulum.handleB();}
// PCI manager interrupt
PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);

float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float setpoint = 0.0, input = 0.0, output = 0.0;
float lastError = 0.0, integral = 0.0;
float dt = 0.01;

void setup() {
  Serial.begin(9600);
  motorSensor.init();
   
   // init the pendulum encoder
  pendulum.init();
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);

  motor.linkSensor(&motorSensor);
  driver.voltage_power_supply = 10;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.init();
  motor.initFOC();
}

void loop() {
  motor.loopFOC();

  pendulum.update();
  input = pendulum.getSensorAngle();
  float error = setpoint - input;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  motor.move(constrain(output, -12.0, 12.0));
  delay((int)(dt * 1000));
  Serial.print("Pendulum Angle: ");
  Serial.print(input);
  Serial.print(" Motor Voltage: ");
  Serial.println(output);
}