import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp

# Parameters
g = 9.81               # Gravitational acceleration (m/s^2)
l = 1.0                # Length of pendulum (m)
m = 1.0                # Mass of pendulum (kg)
b = 0.1                # Damping coefficient
I = (1/3) * m * l**2   # Moment of inertia (for a uniform rod)
motor_time_constant = 0.2  # Motor response time constant
motor_gain = 1.0       # Motor gain

# State-space matrices
A = np.array([
    [0, 1],
    [g/l, -b/I]
])

B = np.array([
    [0],
    [1/I]
])

# Cost matrices for LQR
Q = np.diag([100, 1])  # Penalize angle deviation and angular velocity
R = np.array([[1]])    # Penalize control effort

# Solve the continuous-time algebraic Riccati equation (CARE)
P = solve_continuous_are(A, B, Q, R)

# Calculate the LQR gain matrix
K = np.linalg.inv(R) @ B.T @ P

# Augmented dynamics to include motor delay
def pendulum_dynamics_with_motor(t, state):
    theta, theta_dot, motor_state = state  # Unpack state variables
    motor_output = motor_gain * motor_state  # Motor output
    
    # Pendulum dynamics
    theta_ddot = (g / l * np.sin(theta) - b / I * theta_dot + motor_output / I)
    
    # Motor dynamics (first-order system)
    motor_state_dot = -motor_state / motor_time_constant + -K @ np.array([theta, theta_dot]) / motor_time_constant
    
    return [theta_dot, theta_ddot, motor_state_dot]

# Initial conditions
theta_0 = np.pi + 0.1  # Initial angle (radians, slightly perturbed)
theta_dot_0 = 0.0      # Initial angular velocity (rad/s)
motor_state_0 = 0.0    # Initial motor state
state_0 = [theta_0, theta_dot_0, motor_state_0]

# Time span for simulation
t_span = (0, 10)
t_eval = np.linspace(*t_span, 1000)

# Solve the system
solution = solve_ivp(pendulum_dynamics_with_motor, t_span, state_0, t_eval=t_eval)

# Extract the results
theta = solution.y[0]
theta_dot = solution.y[1]
motor_state = solution.y[2]
time = solution.t

# Plot the results
plt.figure(figsize=(10, 8))
plt.subplot(3, 1, 1)
plt.plot(time, theta, label="Angle (rad)")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, theta_dot, label="Angular Velocity (rad/s)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, motor_state, label="Motor State (V)")
plt.xlabel("Time (s)")
plt.ylabel("Motor Voltage (V)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()