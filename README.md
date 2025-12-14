# CubeSat Attitude Control System
This repository implements an **Attitude Determination and Control System (ADCS)** for a CubeSat using MATLAB & Simulink. The project focuses on stabilizing the spacecraft using reaction wheels with a *LQI controller* and a *Kalman Filter* for state estimation.

![](./Animations/cubesat_translucent.gif)

## Key Features
* **Dynamic Modeling:**
    * Full non-linear rigid body dynamics.
    * Inertia tensor calculation based on geometry (structure + reaction wheels).
* **Control Strategy:**
    * *Discrete LQI Controller:* Optimal control with integral action for zero steady-state error.
    * *Anti-Windup:* Logic to prevent integral accumulation during actuator saturation.
* **Estimation:**
    * *Extended Kalman Filter:* Fuses noisy sensor data to estimate attitude and angular velocity.
    * Realistic sensor noise models (white noise, bias instability, jitter).
* **Simulation:**
    * *Discrete Time Domain:* Controller runs at 100Hz (dt = 0.01s).
    * *Disturbance Rejection:* Tests robustness against external torques (e.g., gravity gradient, micrometeroid impacts).
    * *3D Animation:* Integrated script to visualize the satellite's orientation.

## How to Run
1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/nahuelpucciarelli/Attitude-Control-Cubesat.git](https://github.com/nahuelpucciarelli/Attitude-Control-Cubesat.git)
    ```
2.  **Open MATLAB:**
    R2023b version or later
3.  **Launch the Script:**
    Open `final_project_CyS.mlx` in the `Matlab Files` folder
4.  **Run Simulation:**
    Click *Run All* or *Run Section* in the Live Editor. This will:
    * Load parameters.
    * Linearize the plant.
    * Compute LQI gains.
    * Run the Simulink simulation.
    * Generate plots and animation.

## Configuration & Options
You can customize the simulation directly at the top of the script.

### 1. General Settings (Section: *Simulink Model Options*)
| Variable | Value | Description |
| :--- | :--- | :--- |
| `wheel_corner` | `true` / `false` | **True:** Wheels in corner config. **False:** Wheels aligned on axes. |
| `use_smooth` | `true` / `false` | **True:** Follows a smooth trajectory. **False:** Follows step commands. |
| `save_gif` | `true` / `false` | Set to `true` to save the 3D animation as a `.gif` file. |

### 2. Changing Reference Angles (Section: *Trajectory and Disturbance*)
To change where the satellite points, modify the output vectors (in degrees). Continuous and step disturbances can be added (in Nm).

## Author
*Nahuel Pucciarelli*
