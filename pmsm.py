import numpy as np
import matplotlib.pyplot as plt

# Define motor parameters
L_d = 0.005  # D-axis inductance (H)
L_q = 0.005  # Q-axis inductance (H)
R_s = 0.01   # Stator resistance (ohms)
p = 4        # Number of pole pairs
J = 0.01     # Rotor inertia (kg.m^2)
B = 0.001    # Friction coefficient (Nm.s/rad)
psi_f = 0.1  # Permanent magnet flux linkage (Wb)

# Time settings
dt = 0.0001  # Time step (s)
t_end = 1    # Simulation end time (s)
time = np.arange(0, t_end, dt)

# Initial conditions
i_d = 0.0
i_q = 0.0
omega_r = 0.0  # Rotor speed (rad/s)
theta = 0.0

# PI controller for current control
class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0.0

    def control(self, error, dt):
        self.integral += error * dt
        return self.kp * error + self.ki * self.integral

# PID controller for speed control
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# PMSM dynamics (D-Q axis)
def pmsm_dynamics(i_d, i_q, omega_r, v_d, v_q):
    di_d_dt = (v_d - R_s * i_d + L_q * omega_r * i_q) / L_d
    di_q_dt = (v_q - R_s * i_q - L_d * omega_r * i_d - omega_r * psi_f) / L_q
    return di_d_dt, di_q_dt

# Electromagnetic torque
def compute_torque(i_d, i_q):
    return (3/2) * p * (psi_f * i_q + (L_d - L_q) * i_d * i_q)

# Rotor dynamics (update rotor speed)
def update_omega(omega_r, T_e, T_load, dt):
    d_omega_r_dt = (T_e - T_load - B * omega_r) / J
    return omega_r + d_omega_r_dt * dt

# Define the load torque profile (time-varying load torque)
def load_torque_profile(t):
    if t < 0.4:
        return 0.0  # No load torque before 0.4 seconds
    else:
        return 1.0  # 1 Nm load torque after 0.4 seconds

# Speed reference profile (target rotor speed)
def speed_ref_profile(t):
    if t < 0.6:
        return 100.0  # 100 rad/s speed reference before 0.6 seconds
    else:
        return 150.0  # 150 rad/s speed reference after 0.6 seconds


# Clamping function to limit voltage output
def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

# PI controller gains for current
pi_id = PIController(kp=1.0, ki=100.0)
pi_iq = PIController(kp=1.0, ki=100.0)

# PID controller gains for speed control
pid_speed = PIDController(kp=0.1, ki=1.0, kd=0.01)

# Simulation data storage
time_data = []
i_d_data = []
i_q_data = []
v_d_data = []
v_q_data = []
omega_data = []
torque_data = []
i_d_ref_data = []
i_q_ref_data = []
T_load_data = []
omega_ref_data = []

# Simulation loop
for t in time:
    theta += omega_r * dt

    # Speed reference and load torque profiles
    omega_ref = speed_ref_profile(t)
    T_load = load_torque_profile(t)
    
    # Speed control (determine i_q_ref using PID based on speed error)
    speed_error = omega_ref - omega_r
    i_q_ref = pid_speed.control(speed_error, dt)
    i_d_ref = 0.0  # Keep i_d_ref at zero for decoupling

    # Current control (clamp the control output to avoid overshoot)
    v_d = clamp(pi_id.control(i_d_ref - i_d, dt), -300, 300)
    v_q = clamp(pi_iq.control(i_q_ref - i_q, dt), -300, 300)

    # Update motor currents using PMSM dynamics
    di_d_dt, di_q_dt = pmsm_dynamics(i_d, i_q, omega_r, v_d, v_q)
    i_d += di_d_dt * dt
    i_q += di_q_dt * dt

    # Compute electromagnetic torque
    T_e = compute_torque(i_d, i_q)

    # Update rotor speed using mechanical dynamics
    omega_r = update_omega(omega_r, T_e, T_load, dt)

    # Store data for plotting
    time_data.append(t)
    i_d_data.append(i_d)
    i_q_data.append(i_q)
    v_d_data.append(v_d)
    v_q_data.append(v_q)
    omega_data.append(omega_r)
    torque_data.append(T_e)
    T_load_data.append(T_load)
    omega_ref_data.append(omega_ref)

    # Store reference values
    i_d_ref_data.append(i_d_ref)

# Plotting the results
plt.figure(figsize=(10, 10))

plt.subplot(5, 1, 1)
plt.plot(time_data, i_d_data, label="i_d (D-axis current)")
plt.plot(time_data, i_q_data, label="i_q (Q-axis current)")
plt.plot(time_data, i_d_ref_data, '--', label="i_d_ref (Reference)", alpha=0.7)
plt.title("D-Q Axis Currents")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.legend()

plt.subplot(5, 1, 2)
plt.plot(time_data, v_d_data, label="v_d (D-axis voltage)")
plt.plot(time_data, v_q_data, label="v_q (Q-axis voltage)")
plt.title("D-Q Axis Voltages")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.legend()

plt.subplot(5, 1, 3)
plt.plot(time_data, omega_data, label="Rotor Speed")
plt.plot(time_data, omega_ref_data, '--', label="Reference Speed", alpha=0.7)
plt.title("Rotor Speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (rad/s)")
plt.legend()

plt.subplot(5, 1, 4)
plt.plot(time_data, torque_data, label="Electromagnetic Torque")
plt.title("Electromagnetic Torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.subplot(5, 1, 5)
plt.plot(time_data, T_load_data, label="Load Torque")
plt.title("Load Torque")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()

plt.tight_layout()
plt.show()
