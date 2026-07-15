[Click here to view the 8-minute simulation video explaining the whole process.](https://drive.google.com/file/d/1g53N-A0Kfzxl2ggKPgtg0wd5Ye5gqYjk/view?usp=sharing)

# Estimated-Inverse-Dynamics-Model-Based-CTC_for_RRR-RobotArm


An end-to-end parameter estimation framework and model-based Computed Torque Controller (CTC) for an RRR serial robotic manipulator. Built using Simulink, Simscape Multibody, and MATLAB.
## 📝 Project Overview

Conventional model-free controllers (such as standalone PD loops) suffer from severe tracking degradation when executing high-speed, multi-joint trajectories. Because these reactive loops only respond *after* tracking errors have already developed, they cannot compensate for the heavy, non-linear effects of joint cross-coupling, centrifugal forces, Coriolis acceleration, and gravitational pull. 

Computed Torque Control (CTC) resolves this limitation by linearizing and decoupling the robot's multi-body physics on the fly. By evaluating the robot's inverse dynamic model in real time, CTC allows the arm to move like a collection of independent, weightless particles in space. However, this high-performance architecture is entirely dependent on having an exceptionally accurate plant model. If the underlying mass, center of mass (COM), and inertial parameters are uncalibrated or unknown, the model-based decoupling loop fails, introducing severe instability and calculation distortions.

This project addresses this core bottleneck by establishing an end-to-end **parameter estimation framework and model-based Computed Torque Control (CTC) pipeline** for a 3-DOF RRR serial manipulator. The framework systematically maps the robot's symbolic regressor equations, maximizes parameter visibility via optimized multi-joint excitation paths, and extracts the dynamic parameters using a constrained convex optimization loop. By converting these verified parameters into active programmatic Simulink blocks, the architecture provides a highly precise, decoupled inverse dynamics control loop capable of tracking complex trajectories.

-------------------------------------

<img width="1266" height="670" alt="Screenshot 2026-07-16 011321" src="https://github.com/user-attachments/assets/06543674-fb80-4d7d-a9c2-0d7fda6b6776" />


Figure 1 The parameter excitation experiment.

---------------------------------

<img width="800" height="439" alt="Step_2_ParameterExcitation-ezgif com-optimize" src="https://github.com/user-attachments/assets/b529f8f7-b4ab-464b-b893-ee09694a0f4f" />


Figure 2 the excitation experiment.

---------------------------------

<img width="1906" height="922" alt="Screenshot 2026-07-16 013410" src="https://github.com/user-attachments/assets/cae8653d-8b50-40d8-a585-4d7074f9766a" />


Figure 3 The parameter estimation validation against the real torque. 

---------------------------------

<img width="800" height="439" alt="ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/c88f070f-1618-4b70-ba58-18317e9cb4f2" />


Figure 4  testing the newly estimated parameter against different state variables and torque data.

---------------------------------



<img width="1906" height="926" alt="Screenshot 2026-07-16 014603" src="https://github.com/user-attachments/assets/7a35ba13-3269-4830-baf2-72e7a908f18d" />



Figure 5 The Validation experiment results. 


----------------------------------



<img width="1391" height="576" alt="Screenshot 2026-07-10 215122" src="https://github.com/user-attachments/assets/930f0eb0-365b-439d-b2ae-0aef2609ade8" />

Figure 6 The system architecture after the integeration of CTC.

----------------------------------


<img width="1173" height="703" alt="Screenshot 2026-07-10 230622" src="https://github.com/user-attachments/assets/90299ecb-ed34-49a2-a721-d3bd53845a5f" />

Figure 7 The CTC with the estimated inverse dynamics matrices.


----------------------------

<img width="900" height="641" alt="Screenshot 2026-07-10 181016" src="https://github.com/user-attachments/assets/d17f72b6-960d-47b9-83f9-81829f632898" />

Figure 8 The outer loop configuration. 

---------------------------

<img width="1907" height="915" alt="Screenshot 2026-07-10 220300" src="https://github.com/user-attachments/assets/5d1fdbda-c5dc-4646-916f-a862c65d8343" />

Figure 9 Testing the controller at zero joint position signal (Notice the proactive applied torques in the joint 2 and 3).

---------------------------

<img width="746" height="422" alt="Step_12_TheEstimatedINVDynamicsMatricesCTC" src="https://github.com/user-attachments/assets/072afff9-5418-4800-a183-0289c0b58d83" />

Figure 10 A visualization of the robot stance at zero position state.

--------------------------------


<img width="1906" height="922" alt="Screenshot 2026-07-16 021606" src="https://github.com/user-attachments/assets/18867ac6-a79b-4926-960b-b350af352ede" />


Figure 11 Testing the feedback linearzation. 

--------------------------------

<img width="1905" height="929" alt="Screenshot 2026-07-16 021930" src="https://github.com/user-attachments/assets/915856bc-56a6-467a-afb5-94c43091093b" />


Figure 12 A validation that the robot arm joints are now acting as independent parts after increasing the amplitude of the base joint position signal(notice the tracking error of joint 2 and 3 does not change.

----------------------

<img width="1907" height="910" alt="Screenshot 2026-07-10 233839" src="https://github.com/user-attachments/assets/bdba30af-0fa3-40a2-a39b-ce3c496b9acd" />

Figure 13 An additional validation of the feedback linearzation after increasing the speed of the base joint position signal (The same tracking error in joint 2 and 3).

---------------------------
<img width="1079" height="700" alt="Screenshot 2026-07-10 230831" src="https://github.com/user-attachments/assets/6ba3fd4d-793e-49e4-bf99-82f7a23bd22d" />

Figuere 14 A ground truth validation by applying the real inverse dynamics matrices CTC with same input joint position command as the last validation.

---------------------------


<img width="1907" height="927" alt="Screenshot 2026-07-11 000043" src="https://github.com/user-attachments/assets/2296d005-a440-422c-842b-6b9e28db1a3f" />

Figure 15 The real CTC configuration gives the same results as the estimated CTC. 

----------------------------


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
