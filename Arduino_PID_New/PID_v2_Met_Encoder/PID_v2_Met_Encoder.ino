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
  // Update pendulum sensor
  pendulum.update();

  // Calculate pendulum angle and velocity
  float pendulumAngle = constrainAngle(pendulum.getAngle() + M_PI);
  float pendulumVelocity = pendulum.getVelocity();

  float targetVoltage;
  
  if (abs(pendulumAngle) < 0.5) {
    // Stabilization with LQR control
    targetVoltage = controllerLQR(pendulumAngle, pendulumVelocity, motor.shaftVelocity());
  } else {
    // Swing-up logic
    targetVoltage = calculateSwingUp(pendulumVelocity);
  }

  // Apply the calculated voltage to the motor
  motor.move(targetVoltage);
}


// Constrain the angle between -pi and pi
float constrainAngle(float x) {
  x = fmod(x + M_PI, _2PI);
  if (x < 0) x += _2PI;
  return x - M_PI;
}

// LQR stabilization controller functions
float controllerLQR(float p_angle, float p_vel, float m_vel) {
  // LQR controller: u = k*x
  // k = [40, 7, 0.3]
  float u = 40 * p_angle + 7 * p_vel + 0.3 * m_vel;
  // Limit the voltage
  if (abs(u) > motor.voltage_limit * 0.7) u = _sign(u) * motor.voltage_limit * 0.7;
  return u;
}

// Swing-up controller
float calculateSwingUp(float pendulumVelocity) {
  return -_sign(pendulumVelocity) * motor.voltage_limit * 0.4;
}