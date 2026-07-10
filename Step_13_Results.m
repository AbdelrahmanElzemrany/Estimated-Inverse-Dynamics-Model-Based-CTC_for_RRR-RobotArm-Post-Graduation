% 1. Extract time vectors (assuming modern Dataset or Structure with Time format)
% If you get an error here, check if your workspace contains a single structure named 'out'
time_vec1 = out.Pos1.time; 

% 2. Extract the data arrays [Time x 3 Joints]
desired_pos1 = out.Pos1.data;
torque_appl1 = out.TorqueApplied1.data;
joint_error1 = out.JointPositionError1.data;

% 3. Create a professional, clean multi-plot figure
figure('Color', 'w', 'Name', 'Real CTC Controller Signals');

% --- Subplot 1: Desired Positions ---
subplot(3,1,1);
plot(time_vec, desired_pos, 'LineWidth', 1.5);
grid on;
title('Desired Joint Positions');
ylabel('Position (rad)');
legend('Base', 'Shoulder', 'Wrist', 'Location', 'best');

% --- Subplot 2: Joint Position Errors ---
subplot(3,1,2);
plot(time_vec, joint_error1, 'LineWidth', 1.5);
grid on;
title('Joint Position Tracking Errors');
ylabel('Error (rad)');

% --- Subplot 3: Applied Control Torques ---
subplot(3,1,3);
plot(time_vec1, torque_appl1, 'LineWidth', 1.5);
grid on;
title('Applied Control Torques');
xlabel('Time (seconds)');
ylabel('Torque (Nm)');
