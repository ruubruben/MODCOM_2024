#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

// BLDC motor init
BLDCMotor motor = BLDCMotor(11);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);
//Motor encoder init
Encoder encoder = Encoder(3, 2, 512);
// interrupt routine 
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// pendulum encoder init
Encoder pendulum = Encoder(A0, A1, 1024);
// interrupt routine 
void doPA() { pendulum.handleA(); }
void doPB() { pendulum.handleB(); }
// PCI manager interrupt
PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);

float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float setpoint = 0.0, input = 0.0, output = 0.0;
float lastError = 0.0, integral = 0.0;
float dt = 0.01;

long loop_count = 0; // to track the 25ms wait time
unsigned long lastMillis = 0; // track the time for 25ms interval

void setup() {
  Serial.begin(9600);
  motorSensor.init();
  
  // Init the pendulum encoder
  pendulum.init();
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);

  motor.linkSensor(&motorSensor);
  driver.voltage_power_supply = 10;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity;
  motor.init();
  motor.initFOC();
  
  Serial.println("System Initialized.");
}

void loop() {
  motor.loopFOC();

  // Update pendulum sensor
  pendulum.update();

  // Calculate pendulum angle and velocity
  float pendulumAngle = constrainAngle(pendulum.getAngle() + M_PI);
  float pendulumVelocity = pendulum.getVelocity();
  
  // Control loop every 25ms
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();  // Reset timer
    loop_count++;  // Increment the loop count

    // Print debug information
    Serial.print("Loop Count: ");
    Serial.println(loop_count);
    Serial.print("Pendulum Angle: ");
    Serial.println(pendulumAngle);
    Serial.print("Pendulum Velocity: ");
    Serial.println(pendulumVelocity);
    
    // Determine the target voltage
    float targetVoltage;
    if (abs(pendulumAngle) < 0.5) {
      // Stabilization with LQR control
      targetVoltage = controllerLQR(pendulumAngle, pendulumVelocity, motor.shaftVelocity());
    } else {
      // Swing-up logic
      targetVoltage = calculateSwingUp(pendulumVelocity);
    }

    // Print target voltage
    Serial.print("Target Voltage: ");
    Serial.println(targetVoltage);

    // Apply the calculated voltage to the motor
    motor.move(targetVoltage);
  }
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