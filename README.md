# CubeSat Attitude Control System
This repository implements an **Attitude Determination and Control System (ADCS)** for a CubeSat using MATLAB & Simulink. The project focuses on stabilizing the spacecraft using reaction wheels with a *LQI controller* and a *Kalman Filter* for state estimation.

## Key Features
* **Dynamic Modeling:**
    * Full non-linear rigid body dynamics.
    * Inertia tensor calculation based on geometry (Structure + Reaction Wheels).
    * Steiner theorem application for off-center wheel placement.
* **Control Strategy:**
    * **Discrete LQI Controller:** Optimal control with integral action for zero steady-state error.
    * **Anti-Windup:** Logic to prevent integral accumulation during actuator saturation.
    * **Jacobian Linearization:** Automated linearization of the non-linear plant.
* **Estimation:**
    * **Extended Kalman Filter (EKF):** Fuses noisy sensor data (ADCS, Gyro, Hall Sensors) to estimate attitude and angular velocity.
    * Realistic sensor noise models (White Noise, Bias Instability, Jitter).
* **Simulation:**
    * **Discrete Time Domain:** Controller runs at 100Hz (dt = 0.01s).
    * **Disturbance Rejection:** Tests robustness against external torques (e.g., gravity gradient, micrometeroid impacts).
    * **3D Animation:** Integrated script to visualize the satellite's orientation.

## 🛠️ System Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Mass** | ~2.12 kg | 2U CubeSat Frame + 3 Wheels |
| **Actuators** | 3 Reaction Wheels | Max Torque: 0.6 mNm, Max Speed: 5600 RPM |
| **Sensors** | ADCS, Gyro, Hall | Modeled with Gaussian noise and bias |
| **Controller** | Discrete LQI | Sample Time: 0.01s (100 Hz) |

## How to Run
1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/nahuelpucciarelli/Attitude-Control-Cubesat.git](https://github.com/nahuelpucciarelli/Attitude-Control-Cubesat.git)
    ```
2.  **Open MATLAB:**
    R2023b version or later
3.  **Launch the Script:**
    Open `Matlab Files/final_project_CyS.mlx`.
4.  **Run Simulation:**
    All the files need to be in the same folder.
    Click **Run All** or **Run Section** in the Live Editor. This will:
    * Load parameters.
    * Linearize the plant.
    * Compute LQI gains.
    * Run the Simulink simulation.
    * Generate plots and animation.

## Configuration & Options
You can customize the simulation directly at the top of the `final_project_CyS.mlx` script.

### 1. General Settings (Section: *Simulink Model Options*)
| Variable | Value | Description |
| :--- | :--- | :--- |
| `wheel_corner` | `true` / `false` | **True:** Wheels in corner config. **False:** Wheels aligned on axes. |
| `use_smooth` | `true` / `false` | **True:** Follows a smooth trajectory. **False:** Follows step commands. |
| `save_gif` | `true` / `false` | Set to `true` to save the 3D animation as a `.gif` file. |

### 2. Changing Reference Angles (Section: *Trajectory and Disturbance*)
To change where the satellite points, modify the vectors (in degrees). Continuous and step disturbances can be added (in Nm).

## Author
*Nahuel Pucciarelli*
