%Created By Abdelrahman Elzemrany
Beta2=theta;
clc
k=21;
time = 10;   %Experiment time
T = 0.001;   %Sampling period
l = time/T;  %Number of samples
q11=pos1.signals(1).values(1:l) ;  %Joint position 1
q22=pos1.signals(2).values(1:l);   %Joint position 2
q33=pos1.signals(3).values(1:l);  
qp11=velocity1.signals(1).values(1:l);  
qp22= velocity1.signals(2).values(1:l);  
qp33=velocity1.signals(3).values(1:l);  
qpp11=accelration1.signals(1).values(1:l) ;   %joint accelration1
qpp22= accelration1.signals(2).values(1:l) ;  
qpp33=accelration1.signals(3).values(1:l) ; 
u11=torque1.signals(1).values(1:l);   %Voltage 1
u22=torque1.signals(2).values(1:l); %Voltage 2
u33=torque1.signals(3).values(1:l); 
tau11 = u11; %Torque 1
tau22 = u22; %Torque 2
tau33=u33*1;
t=pos1.time(1:10000,1);
%mass
for i=1:10000
   Yb2(:,:,i)=Y_b_handle(0,0,-9.8,q11(i),q22(i),q33(i),qp11(i),qp22(i),qp33(i),qpp11(i),qpp22(i),qpp33(i));
end
for ii=1:k
    for ik=1:l
        sum11(ik,ii)=Yb2(1,ii,ik);
        sum22(ik,ii)=Yb2(2,ii,ik);
       sum33(ik,ii)=Yb2(3,ii,ik);
     
    end
end
Yc2=[sum11;sum22;sum33];
tv_1=Yc2*theta ;
tv11=tv_1(1:10000,1);
tv22=tv_1(10001:20000,1);
tv33=tv_1(20001:30000,1);
tv11t=timeseries(tv11);
tv22t=timeseries(tv22);
tv33t=timeseries(tv33);
tau11t=timeseries(tau11);
tau22t=timeseries(tau22);
tau33t=timeseries(tau33);
%% ==================== METRIC CALCULATION FOR EXP 2 ====================
l_total = length(tau11); % 10000 samples

% Combine validation measurements and estimations into single system vectors
tt2 = [tau11; tau22; tau33];
tv2_all = [tv11; tv22; tv33];

% --- Joint 1 (Base Yaw) Validation Accuracy ---
rmse1 = sqrt(mean((tau11 - tv11).^2));
denom1 = max(tau11) - min(tau11);
if denom1 == 0, denom1 = 1; end % Prevent division by zero if joint didn't move
fit1 = (1 - (rmse1 / denom1)) * 100;

% --- Joint 2 (Shoulder Pitch) Validation Accuracy ---
rmse2 = sqrt(mean((tau22 - tv22).^2));
denom2 = max(tau22) - min(tau22);
if denom2 == 0, denom2 = 1; end
fit2 = (1 - (rmse2 / denom2)) * 100;

% --- Joint 3 (Elbow Pitch) Validation Accuracy ---
rmse3 = sqrt(mean((tau33 - tv33).^2));
denom3 = max(tau33) - min(tau33);
if denom3 == 0, denom3 = 1; end
fit3 = (1 - (rmse3 / denom3)) * 100;

% --- Overall Cross-Validation System Accuracy ---
rmse_total = sqrt(mean((tt2 - tv2_all).^2));
denom_total = max(tt2) - min(tt2);
fit_overall = (1 - (rmse_total / denom_total)) * 100;

% --- Statistical R-squared (R2) Score for Validation ---
SS_res = sum((tt2 - tv2_all).^2);
SS_tot = sum((tt2 - mean(tt2)).^2);
R_squared = (1 - (SS_res / SS_tot)) * 100;

%% ==================== PRINT VALIDATION RESULTS ====================
fprintf('\n================== VALIDATION EXPERIMENT 2 ==================\n');
fprintf('Joint 1 (Base Yaw)   Fit Percentage: %6.2f %%\n', fit1);
fprintf('Joint 2 (Shoulder)   Fit Percentage: %6.2f %%\n', fit2);
fprintf('Joint 3 (Elbow)      Fit Percentage: %6.2f %%\n', fit3);
fprintf('-------------------------------------------------------------\n');
fprintf('OVERALL CROSS-VALIDATION ACCURACY:  %6.2f %%\n', fit_overall);
fprintf('Statistical Validation R2 Score:    %6.4f %%\n', R_squared);
fprintf('=============================================================\n');


