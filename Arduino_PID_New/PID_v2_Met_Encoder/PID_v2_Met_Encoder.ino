#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>

// Motor and Driver Configuration
const int motorPolePairs = 11;
const float driverVoltage = 12.0;
const float voltageLimit = driverVoltage * 0.7;

BLDCMotor motor = BLDCMotor(motorPolePairs);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

// Motor Encoder Configuration
Encoder motorEncoder = Encoder(2, 3, 500);
void handleMotorEncoderA() { motorEncoder.handleA(); }
void handleMotorEncoderB() { motorEncoder.handleB(); }

// Pendulum Encoder Configuration
Encoder pendulumEncoder = Encoder(A1, A2, 1000);
void handlePendulumEncoderA() { pendulumEncoder.handleA(); }
void handlePendulumEncoderB() { pendulumEncoder.handleB(); }
PciListenerImp pendulumListenerA(pendulumEncoder.pinA, handlePendulumEncoderA);
PciListenerImp pendulumListenerB(pendulumEncoder.pinB, handlePendulumEncoderB);

// Control Parameters
long loopCounter = 0;
const int controlInterval = 25; // Run control every ~25ms
float targetVoltage = 0;

void setup() {
    // Initialize Serial for Debugging
    Serial.begin(115200);

    // Initialize Motor Encoder
    motorEncoder.init();
    motorEncoder.enableInterrupts(handleMotorEncoderA, handleMotorEncoderB);

    // Initialize Pendulum Encoder
    pendulumEncoder.init();
    PciManager.registerListener(&pendulumListenerA);
    PciManager.registerListener(&pendulumListenerB);

    // Motor Configuration
    motor.controller = MotionControlType::torque;
    motor.linkSensor(&motorEncoder);
    
    // Driver Configuration
    driver.voltage_power_supply = driverVoltage;
    driver.init();
    motor.linkDriver(&driver);

    // Initialize Motor
    motor.init();
    motor.voltage_limit = voltageLimit;
    motor.initFOC();

    Serial.println("System Initialized");
}

void loop() {
    // Update motor FOC loop (~1ms)
    motor.loopFOC();

    // Run control logic every ~25ms
    if (loopCounter++ >= controlInterval) {
        pendulumEncoder.update();

        // Calculate pendulum angle
        float pendulumAngle = constrainAngle(pendulumEncoder.getAngle() + M_PI);

        // Determine control action
        if (abs(pendulumAngle) < 0.5) {
            // Stabilize the pendulum
            targetVoltage = calculateLQR(pendulumAngle, pendulumEncoder.getVelocity(), motor.shaftVelocity());
        } else {
            // Swing-up logic
            targetVoltage = calculateSwingUp(pendulumEncoder.getVelocity());
        }

        // Apply voltage to the motor
        motor.move(targetVoltage);

        // Reset loop counter
        loopCounter = 0;
    }
}

// Helper Function to Constrain Angle Between -pi and pi
float constrainAngle(float angle) {
    angle = fmod(angle + M_PI, _2PI);
    if (angle < 0) {
        angle += _2PI;
    }
    return angle - M_PI;
}

// LQR Controller Implementation
float calculateLQR(float pendulumAngle, float pendulumVelocity, float motorVelocity) {
    // LQR Gains
    const float kPendulumAngle = 40.0;
    const float kPendulumVelocity = 7.0;
    const float kMotorVelocity = 0.3;

    // Calculate control input
    float controlSignal = kPendulumAngle * pendulumAngle + kPendulumVelocity * pendulumVelocity + kMotorVelocity * motorVelocity;

    // Limit voltage
    return constrain(controlSignal, -voltageLimit, voltageLimit);
}

// Swing-Up Controller Implementation
float calculateSwingUp(float pendulumVelocity) {
    float swingVoltage = -_sign(pendulumVelocity) * voltageLimit * 0.4;
    return swingVoltage;
}