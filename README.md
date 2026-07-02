[Click here and press view raw to download and view the 8-minute simulation video explaining the whole process.](PASTE_YOUR_GOOGLE_DRIVE_LINK_HERE) 

# 3r-rrr-robot-estimated-inverse-dynamics-model-based-ctc

An end-to-end parameter estimation framework and model-based Computed Torque Controller (CTC) for a 3R RRR serial robotic manipulator. Built using Simulink, Simscape Multibody, and MATLAB.

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
* **`Step_8_AdditionalValidatation.slx`**: Visual scope environment that matches the true validation torques directly against the estimated curves.
* **`Step_9_ReformTheEstimatedMatrices.m`**: Algebraically reconstructs the standalone estimated Mass, Coriolis, and Gravity matrices.
* **`Step_10_CheckPositive.m`**: Runs a safety script to audit the unconstrained model across 27,000 distinct operational joint configurations.

### 5. Model-Based Control Integration
* **`Step_11_GetTheEstimatedINVdynamics.m`**: Executes the `matlabFunctionBlock` utility to automatically generate native Simulink block code.
* **`Step_12_TheEstimatedINVDynamicsMatricesCTC.slx`**: Integration canvas where previous block placeholders are swapped out for the new estimation subsystems.
* **`Step_13_Real_RRR_CTC_validation.slx`**: Evaluates the completed Computed Torque Controller (CTC) loop under high-speed trajectory stress testing.

---

> [!IMPORTANT]
> ### 🛡️ Workspace Matrix Inversion & Safety Audit Note
> 
> To ensure the Computed Torque Controller remains fully stable without encountering division spikes, the estimated inertia matrix must maintain strict positive-definiteness ($M(q) > 0$). To verify this across the entire operational workspace, Step 10 conducts a comprehensive configuration scan:
> 
> * **High-density safety auditing** evaluated the minimum matrix eigenvalues across a test grid of **27,000 discrete positions**.
> * The system verified a worst-case minimum eigenvalue of **0.001483**, mathematically confirming that the custom blocks are safe from matrix calculation errors.
