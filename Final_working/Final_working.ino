#include <SimpleFOC.h>
// software interrupt library
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
Encoder pendulum = Encoder(10, 11, 512);
// interrupt routine 
void doPA(){pendulum.handleA();}
void doPB(){pendulum.handleB();}
// PCI manager interrupt
PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);

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

// loop downsampling counter
long loop_count = 0;

void loop() {
  // ~1ms 
  motor.loopFOC();

  // control loop each ~25ms
  if(loop_count++ > 25){
    // updating the pendulum angle sensor
    // NECESSARY for library versions > v2.2 
    pendulum.update();
    // calculate the pendulum angle 
    float pendulum_angle = constrainAngle(pendulum.getAngle() + M_PI);

    float target_voltage;
    if( abs(pendulum_angle) < 0.5 ) // if angle small enough stabilize
      target_voltage = controllerLQR(pendulum_angle, pendulum.getVelocity(), motor.shaftVelocity());
    else // else do swing-up
      // sets 40% of the maximal voltage to the motor in order to swing up
      target_voltage = -_sign(pendulum.getVelocity())*motor.voltage_limit*0.4;

    // set the target voltage to the motor
    motor.move(target_voltage);
    Serial.print("Pendulum Angle: ");
    Serial.print(pendulum_angle);
    Serial.print(" Motor Voltage: ");
    Serial.println(target_voltage);

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