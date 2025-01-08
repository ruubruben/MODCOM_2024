import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Parameters of the inverted pendulum and motor system
g = 9.81  # gravitational acceleration (m/s^2)
L = 0.5   # length of the pendulum (m)
m = 0.1   # mass of the pendulum (kg)
b = 0.05  # damping coefficient (Nm/s)
I = m * L**2 / 3  # moment of inertia of pendulum (kg.m^2)
motor_voltage_limit = 12  # motor voltage limit (V)

# LQR stabilization gains
Kp = 40  # proportional gain
Kd = 7   # derivative gain
Kv = 0.3  # motor velocity gain

# PID swing-up gains
swing_Kp = 10.0  # proportional gain for swing-up
swing_Ki = 0.1   # integral gain for swing-up
swing_Kd = 5.0   # derivative gain for swing-up
swing_integral = 0.0  # integral term initialization

# Helper functions
def constrain_angle(x):
    """Constrain angle to be within -pi and pi."""
    x = np.fmod(x + np.pi, 2 * np.pi)
    if x < 0:
        x += 2 * np.pi
    return x - np.pi


def lqr_control(theta, theta_dot, motor_velocity):
    """LQR control law to calculate motor voltage."""
    u = Kp * theta + Kd * theta_dot + Kv * motor_velocity
    u = np.clip(u, -motor_voltage_limit * 0.7, motor_voltage_limit * 0.7)
    return u


def pid_swing_up_control(theta, theta_dot, dt):
    """PID control for swing-up, returns individual PID components."""
    global swing_integral

    # Compute error as the difference from the upright position
    error = np.pi - abs(theta)

    # Update integral term
    swing_integral += error * dt

    # Compute derivative term
    derivative = -theta_dot

    # Compute individual terms
    proportional_term = swing_Kp * error
    integral_term = swing_Ki * swing_integral
    derivative_term = swing_Kd * derivative

    # PID control law
    u = proportional_term + integral_term + derivative_term
    u = np.clip(u, -motor_voltage_limit, motor_voltage_limit)
    
    return u, proportional_term, integral_term, derivative_term

# Initialize arrays to store PID terms
pid_proportional = None
pid_integral = None
pid_derivative = None

def pendulum_dynamics(t, y):
    """Dynamics of the pendulum and motor system."""
    global last_t, pid_proportional, pid_integral, pid_derivative
    theta, theta_dot, motor_velocity = y

    # Time step for PID integral and derivative
    dt = t - last_t if last_t is not None else 0.01
    last_t = t

    # Swing-up or stabilization
    if abs(theta) < 0.5:  # Stabilization region
        motor_voltage = lqr_control(theta, theta_dot, motor_velocity)
        proportional_term = integral_term = derivative_term = 0.0
    else:  # Swing-up region
        motor_voltage, proportional_term, integral_term, derivative_term = pid_swing_up_control(theta, theta_dot, dt)

    # Update PID arrays with current values
    index = np.argmin(np.abs(sol_t - t))  # Use nearest index
    pid_proportional[index] = proportional_term
    pid_integral[index] = integral_term
    pid_derivative[index] = derivative_term

    # Pendulum dynamics
    theta_ddot = (m * g * L * np.sin(theta) - b * theta_dot) / I

    # Simple motor dynamics (assuming linear response for simplicity)
    motor_velocity_dot = (motor_voltage - motor_velocity) / 0.1  # motor time constant = 0.1s

    return [theta_dot, theta_ddot, motor_velocity_dot]

# Simulation setup
y0 = [np.pi / 2, 0.0, 0.0]  # initial state: [angle, angular velocity, motor velocity]
t_span = (0, 10)  # simulate for 10 seconds
sol_t = np.linspace(t_span[0], t_span[1], 1000)  # time points for evaluation
last_t = None  # Initialize last_t for PID control

# Initialize PID arrays
pid_proportional = np.zeros_like(sol_t)
pid_integral = np.zeros_like(sol_t)
pid_derivative = np.zeros_like(sol_t)

# Run the simulation
sol = solve_ivp(pendulum_dynamics, t_span, y0, t_eval=sol_t)

# Plotting results
plt.figure(figsize=(12, 10))

# Pendulum angle
plt.subplot(5, 1, 1)
plt.plot(sol.t, sol.y[0], label="Angle (rad)")
plt.axhline(np.pi, color="red", linestyle="--", label="Upright position")
plt.ylabel("Angle (rad)")
plt.legend()

# Pendulum angular velocity
plt.subplot(5, 1, 2)
plt.plot(sol.t, sol.y[1], label="Angular Velocity (rad/s)")
plt.ylabel("Angular Velocity")
plt.legend()

# Motor velocity
plt.subplot(5, 1, 3)
plt.plot(sol.t, sol.y[2], label="Motor Velocity (rad/s)")
plt.ylabel("Motor Velocity")
plt.legend()

# Motor voltage (control signal)
motor_voltage = [
    lqr_control(th, th_dot, mv) if abs(th) < 0.5 else pid_swing_up_control(th, th_dot, 0.01)[0]
    for th, th_dot, mv in zip(sol.y[0], sol.y[1], sol.y[2])
]
plt.subplot(5, 1, 4)
plt.plot(sol.t, motor_voltage, label="Motor Voltage (V)")
plt.ylabel("Motor Voltage (V)")
plt.legend()

# PID terms
plt.subplot(5, 1, 5)
plt.plot(sol_t, pid_proportional, label="Proportional Term (V)")
plt.plot(sol_t, pid_integral, label="Integral Term (V)")
plt.plot(sol_t, pid_derivative, label="Derivative Term (V)")
plt.ylabel("PID Terms (V)")
plt.xlabel("Time (s)")
plt.legend()

plt.tight_layout()
plt.show()