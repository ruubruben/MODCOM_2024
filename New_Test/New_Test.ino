#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

// Magnetic sensor initialization
MagneticSensorSPI motorSensor = MagneticSensorSPI(AS5048_SPI, 7);  // using the SPI interface and the chip select pin (7)

// BLDC motor init
BLDCMotor motor = BLDCMotor(12);  // Updated motor instance
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 9, 6, 8);  // Updated PWM driver pins

// pendulum encoder init
Encoder pendulum = Encoder(A0, A1, 1024);
// interrupt routine 
void doPA(){pendulum.handleA();}
void doPB(){pendulum.handleB();}
// PCI manager interrupt
PciListenerImp listenerPA(pendulum.pinA, doPA);
PciListenerImp listenerPB(pendulum.pinB, doPB);


void setup() {
  Serial.begin(115200);
  // Initialize the motor sensor (MagneticSensorSPI)
  motorSensor.init();

  // Set up the motor to use the new sensor (MagneticSensorSPI)
  motor.linkSensor(&motorSensor);  // Link the motor to the magnetic sensor

  // Initialize the pendulum encoder
  pendulum.init();
  PciManager.registerListener(&listenerPA);
  PciManager.registerListener(&listenerPB);
  
  // Set control loop type to be used
  motor.controller = MotionControlType::torque;
 //motor.torque_controller =TorqueControlType::foc_current;

  // Driver setup
  driver.voltage_power_supply = 10; 
  driver.init();
  
  // Link the driver and the motor
  motor.linkDriver(&driver);

  // Initialize motor and align with the sensor
  motor.init();
  motor.initFOC();
}

void loop() {

  i

}