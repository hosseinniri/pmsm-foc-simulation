# PMSM FOC Simulation

This project simulates a **Permanent Magnet Synchronous Motor (PMSM)** under **Field-Oriented Control (FOC)** using Python. The simulation includes a PID-based speed controller, dynamic load torque profile, and visualizations for key motor variables like current, voltage, speed, and torque.

## Table of Contents
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Customizing Simulation Parameters](#customizing-simulation-parameters)
- [Simulation Outputs](#simulation-outputs)
- [License](#license)

## Features

- **Field-Oriented Control (FOC)** implementation for a PMSM.
- **PID-based speed controller** to regulate motor speed.
- Customizable **load torque** and **speed reference profiles**.
- Visualization of D-Q axis currents, voltages, rotor speed, and torque responses.
- **Time-varying load torque** and **reference speed** profiles to simulate real-world conditions.

## Installation

Clone the repository and install the required Python libraries:

```bash
git clone https://github.com/your-username/pmsm-foc-simulation.git
cd pmsm-foc-simulation
pip install numpy matplotlib
```

## Usage
To run the simulation, navigate to the project folder and execute the script:

```bash
python pmsm.py
```

The script simulates the PMSM under FOC control and generates performance plots.

Customizing Simulation Parameters
You can modify various motor parameters and simulation settings in the script:

Motor parameters such as inductance, resistance, pole pairs, inertia, friction coefficient, and flux linkage can be modified directly in the code.
The load torque and speed reference profiles can be adjusted in the load_torque_profile and speed_ref_profile functions to simulate different real-world scenarios.
Simulation Outputs
The simulation provides the following plots:

D-Q Axis Currents (i_d, i_q)
D-Q Axis Voltages (v_d, v_q)
Rotor Speed (ω_r)
Electromagnetic Torque (T_e)
Load Torque
These plots will show the motor’s performance under varying load torque and speed reference conditions.

## License
This project is licensed under the MIT License. See the LICENSE file for details.