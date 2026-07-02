## 🚀 Key Pipeline Features
- **Blind Data Collection:** Raw joint Position, Velocity, Acceleration (PVA), and motor torque ($\tau$) data collected completely blind to internal physical properties to guarantee zero cheating.
- **Minimal Optimization Setup:** Reduces the standard dynamics regressor matrix to a minimal base parameter vector ($\beta$) using DH parameters, Jacobians, and QR Decomposition.
- **Physical Feasibility Constraints:** Implements a convex optimization routine via `fmincon` using an auditing grid of 27,000 configurations.
- **Safety Verification:** Checks the system's positive-definiteness to enforce $M(q) > 0$, protecting the CTC loop from mathematical division spikes.

## 📊 Performance Statistics
- **Overall Constrained Parameter Accuracy:** **99.95% Fit**
- **Minimum Audited Workspace Eigenvalue:** **0.001483** (Strictly Positive-Definite Verified)
- **R² Validation Score:** **99.86%**

## 🎥 Full Video Demonstration (8 Minutes)

Watch the complete, end-to-end walkthrough of the pipeline—from raw data extraction to the final high-speed controller stress test—directly on Google Drive:

▶️ [**Watch the Full Project Video on Google Drive**](https://drive.google.com/drive/u/0/folders/1m-NyGNsKJ3eKfLIFNobjmQB3yitPuI5l)

### 📌 Video Chapters
- **0:00 - 2:00** | Step 0 & 2: Model setup, blind excitation trajectory using different sine waves, and logging raw PVA readings.
- **2:00 - 4:00** | Step 1, 3, & 4: Custom regressor matrix assembly and running the constrained `fmincon` optimizer.
- **4:00 - 6:00** | Step 5 - 10: Cross-validation tracking using an independent dataset, torque trajectory overlays, and the 27,000 grid eigenvalue safety audit ($M(q) > 0$).
- **6:00 - 8:00** | Step 11 - 13: Auto-generating native blocks via `matlabFunctionBlock`, building the CTC loop, and final high-speed tracking error analysis.

## 🦾 Controller Performance (Custom CTC vs. Built-in Blocks)
Under an extreme, high-speed tracking stress test, the custom-estimated dynamics loop successfully feedback-linearizes the nonlinear multi-link cross couplings. 
- **Joint 1 (Base Yaw):** The built-in Simulink robotics block provides slightly superior position tracking precision.
- **Joints 2 & 3 (Shoulder/Elbow):** Position tracking profiles between our custom matrix and the built-in ground truth blocks are entirely identical.
- **The Achievement:** Achieving matching tracking performance against massive, optimized internal physics engines proves that your analytical derivation, coding, and parameter estimation are 100% correct.

## 🛠️ Step-by-Step Code Execution
1. Run `Step_0_Model_Initialization.m` to load the true reference workspace variables that the built-in validation blocks inherit.
2. Run `GetTheRegressorMatrix_step1.m` to generate the structural analytics.
3. Run `UR3Excitation_step2.slx` and `UR3DataExtraction_step3.slx` to design the trajectory and extract cross-validation data.
4. Execute `BaseParameterEstimation_step4.m` to extract the calculated $\beta$ values.
5. Execute `ReformulatingTheEstimatedMatrices_step5.m` and `GetTheEstimatedINVDynamics_step6.m` to isolate matrices by toggling states and auto-generate native blocks.
6. Open `UR3G_Matrix_testing_step7.slx` and `Gravity_compensationController_step8.slx` to observe the final high-speed tracking behavior and torque matching on the scopes.
