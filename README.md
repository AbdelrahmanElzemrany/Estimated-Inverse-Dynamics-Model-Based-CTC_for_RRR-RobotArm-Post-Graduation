[Click here to view the 8-minute simulation video explaining the whole process.](https://drive.google.com/file/d/1g53N-A0Kfzxl2ggKPgtg0wd5Ye5gqYjk/view?usp=sharing)

# Estimated-Inverse-Dynamics-Model-Based-CTC_for_RRR-RobotArm


An end-to-end parameter estimation framework and model-based Computed Torque Controller (CTC) for a 3R RRR serial robotic manipulator. Built using Simulink, Simscape Multibody, and MATLAB.
## 📝 Project Overview

Conventional model-free controllers (such as standalone PD loops) suffer from severe tracking degradation when executing high-speed, multi-joint trajectories. Because these reactive loops only respond *after* tracking errors have already developed, they cannot compensate for the heavy, non-linear effects of joint cross-coupling, centrifugal forces, Coriolis acceleration, and gravitational pull. 

Computed Torque Control (CTC) resolves this limitation by linearizing and decoupling the robot's multi-body physics on the fly. By evaluating the robot's inverse dynamic model in real time, CTC allows the arm to move like a collection of independent, weightless particles in space. However, this high-performance architecture is entirely dependent on having an exceptionally accurate plant model. If the underlying mass, center of mass (COM), and inertial parameters are uncalibrated or unknown, the model-based decoupling loop fails, introducing severe instability and calculation distortions.

This project addresses this core bottleneck by establishing an end-to-end **parameter estimation framework and model-based Computed Torque Control (CTC) pipeline** for a 3-DOF RRR serial manipulator. The framework systematically maps the robot's symbolic regressor equations, maximizes parameter visibility via optimized multi-joint excitation paths, and extracts the dynamic parameters using a constrained convex optimization loop. By converting these verified parameters into active programmatic Simulink blocks, the architecture provides a highly precise, decoupled inverse dynamics control loop capable of tracking complex trajectories with over 99.95% global accuracy.

-------------------------------------

<img width="1261" height="692" alt="Screenshot 2026-07-10 134248" src="https://github.com/user-attachments/assets/d672daa7-b40e-488b-9b67-4d32d16c395a" />

---------------------------------

<img width="746" height="422" alt="Step_2_ParameterExcitation (2)" src="https://github.com/user-attachments/assets/8c68d93c-ee2f-49c1-b468-a8a7043f09e4" />


---------------------------------

<img width="1907" height="909" alt="Screenshot 2026-07-10 164246" src="https://github.com/user-attachments/assets/34ee5048-7071-4218-a6d5-2c351007fbac" />


---------------------------------

<img width="746" height="422" alt="Step_5_TestingTheEstimatedParameters (1)" src="https://github.com/user-attachments/assets/078e903f-6185-4843-8907-af4c43e70016" />


---------------------------------


<img width="1907" height="927" alt="Screenshot 2026-07-10 164548" src="https://github.com/user-attachments/assets/af7060c8-823f-4418-9790-fdeb919ca276" />




----------------------------------

<img width="1284" height="729" alt="Screenshot 2026-07-10 180858" src="https://github.com/user-attachments/assets/4108814c-bcf8-4740-a4bc-77d80b165b21" />

----------------------------

<img width="900" height="641" alt="Screenshot 2026-07-10 181016" src="https://github.com/user-attachments/assets/d17f72b6-960d-47b9-83f9-81829f632898" />


## 🛠️ Pipeline & File Architecture

The repository is structured sequentially to take the robot from raw DH parameters to a verified trajectory-tracking controller:

### 1. Kinematic & Symbolic Modeling
* **`Step_0_RRR_import.m`**: Loads the true reference workspace parameters that the built-in system blocks will later inherit as a baseline.
* **`Step_1_GetTheRegressorMatrix.m`**: Generates the structural analytical equations of motion and separates kinematic states from inertial variables.

### 2. Trajectory Generation & Data Extraction
* **`Step_2_ParameterExcitation.slx`**: Excitation trajectory simulation running overlapping multi-joint sine waves to maximize parameter visibility.
* **`Step_3_DataextractForParameterEstimation.slx`**: Logs the resulting joint kinematic states—Position, Velocity, and Acceleration (PVA)—along with motor torques ($\tau$).

### 3. Dynamics & Identification
* **`Step_4_ParameterEstimation.m`**: Runs a convex parameter optimization loop via `fmincon` matching raw PVA data against the symbolic model.
* **`Step_5_TestingTheEstimatedParameters.slx`**: Simulates the estimated parameters against a completely new, independent tracking dataset.
* **`Step_6_DataextractForSecondExpValidation.slx`**: Exports the secondary cross-validation state trajectories back to the MATLAB workspace.

### 4. Verification & Validation
* **`Step_7_ValidationOfEstimation.m`**: Processes the independent datasets to compute global tracking fitness metrics (overall 99.95% accuracy).
* **`Step_8_AdditionalValidation.slx`**: Visual scope environment that matches the true validation torques directly against the estimated curves.
* **`Step_9_ReformTheEstimatedMatrices.m`**: Algebraically reconstructs the standalone estimated Mass, Coriolis, and Gravity matrices.
* **`Step_10_CheckPositive.m`**: Runs a safety script to audit the unconstrained model across 27,000 distinct operational joint configurations.

### 5. Model-Based Control Integration
* **`Step_11_GetTheEstimatedINVdynamics.m`**: Executes the matlabFunctionBlock utility to automatically generate programmatic Simulink blocks from the identified symbolic dynamics equations.
* **`Step_12_TheEstimatedINVDynamicsMatricesCTC.slx`**: Integration canvas where previous block placeholders are swapped out for the new estimation subsystems.
* **`Step_13_Real_RRR_CTC_validation.slx`**: To compare the new estimated blocks against ground truth simulink blocks which include the full information about the tested RRR robot

---

> [!IMPORTANT]
> ### 🛡️ Workspace Matrix Inversion & Safety Audit Note
> 
> To ensure the Computed Torque Controller remains fully stable without encountering division spikes, the estimated inertia matrix must maintain strict positive-definiteness ($M(q) > 0$). To verify this across the entire operational workspace, Step 10 conducts a comprehensive configuration scan:
> 
> * **High-density safety auditing** evaluated the minimum matrix eigenvalues across a test grid of **27,000 discrete positions**.
> * The system verified a worst-case minimum eigenvalue of **0.001483**, mathematically confirming that the custom blocks are safe from matrix calculation errors.
