#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

// Magnetic sensor initialization
MagneticSensorSPI motorSensor = MagneticSensorSPI(AS5048_SPI, 7);  // using the SPI interface and the chip select pin (7)

// BLDC motor init
BLDCMotor motor = BLDCMotor(14);  // Updated motor instance
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);  // Updated PWM driver pins

// pendulum encoder init
Encoder pendulum = Encoder(A0, A1, 512);
// interrupt routine 
void doPA(){pendulum.handleA();}
void doPB(){pendulum.handleB();}
// PCI manager interrupt
PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);

void setup() {
  // Initialize the motor sensor (MagneticSensorSPI)
  Serial.begin(115200);

  motorSensor.init();

  // Set up the motor to use the new sensor (MagneticSensorSPI)
  motor.linkSensor(&motorSensor);  // Link the motor to the magnetic sensor

  // Initialize the pendulum encoder
  pendulum.init();
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);
  
  // Set control loop type to be used
  motor.controller = MotionControlType::torque;

  // Driver setup
  driver.voltage_power_supply = 12; 
  driver.init();
  
  // Link the driver and the motor
  motor.linkDriver(&driver);

  // Initialize motor and align with the sensor
  motor.init();
  motor.initFOC();
}

// loop downsampling counter
long loop_count = 0;
float target_voltage;

void loop() {
  // ~1ms 
  motor.loopFOC();

  // control loop each ~25ms
  if(loop_count++ > 25){
    // Update the pendulum angle sensor
    // NECESSARY for library versions > v2.2 
    pendulum.update();
    
    // Calculate the pendulum angle 
    float pendulum_angle = constrainAngle(pendulum.getAngle() + M_PI);

    if( abs(pendulum_angle) < 0.5 ) // if angle small enough stabilize
      target_voltage = controllerLQR(pendulum_angle, pendulum.getVelocity(), motor.shaftVelocity());
    else // else do swing-up
      target_voltage = -_sign(pendulum.getVelocity())*motor.voltage_limit*0.4;
    Serial.print("Pendulum Angle: ");
    Serial.print(pendulum_angle);
    Serial.print(" Motor Voltage: ");
    Serial.println(target_voltage);
    // Set the target voltage to the motor
    motor.move(target_voltage);

    // restart the counter
    loop_count=0;
  }
}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

// LQR stabilization controller functions
// calculating the voltage that needs to be set to the motor in order to stabilize the pendulum
float controllerLQR(float p_angle, float p_vel, float m_vel){
  // if angle controllable
  // calculate the control law 
  // LQR controller u = k*x
  //  - k = [40, 7, 0.3]
  //  - x = [pendulum angle, pendulum velocity, motor velocity]' 
  float u =  40*p_angle + 7*p_vel + 0.3*m_vel;
  
  // limit the voltage set to the motor
  if(abs(u) > motor.voltage_limit*0.7) u = _sign(u)*motor.voltage_limit*0.7;
  
  return u;
}