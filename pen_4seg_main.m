% Main script: define initial conditions and run ode45 for pen_4seg

clearvars
clc
close all

%% ================= INITIAL CONDITIONS =================

% ---------------- Initial orientations (degrees → radians) ----------------
psi1 = 225*pi/180;   % segment 1
psi2 = 210*pi/180;   % segment 2
psi3 = 195*pi/180;   % segment 3
psiD = 195*pi/180;   % drogue

% --------------- Euler-angle vectors [phi, theta, psi] ------------------
th1 = [0, 0, psi1];
th2 = [0, 0, psi2];
th3 = [0, 0, psi3];
thD = [0, 0, psiD];

% ---------------- Body angular velocities ----------------
omg1 = [0, 0, 0];
omg2 = [0, 0, 0];
omg3 = [0, 0, 0];
omgD = [0, 0, 0];

%% ================= HINGE INITIAL CONDITIONS =================

theta_j1_0 = psi2 - psi1;    % hinge between seg 1–2
theta_j2_0 = psi3 - psi2;    % hinge between seg 2–3
theta_j3_0 = psiD - psi3;    % hinge between seg 3–drogue

thjd1 = 0;
thjd2 = 0;
thjd3 = 0;

%% ================= STATE VECTOR =================

State_ini = [ ...
    th1, omg1, ...
    th2, omg2, ...
    th3, omg3, ...
    thD, omgD, ...
    theta_j1_0, theta_j2_0, theta_j3_0, ...
    thjd1, thjd2, thjd3, ...
    0,0,0, 0,0,0 ];

%% ================= ODE SOLVER =================

ops = odeset('RelTol',1e-1,'AbsTol',1e-1);

[t,z] = ode45(@pen_4seg, 0:0.01:20, State_ini, ops);

%% ================= FINAL ANGLES (last frame) =================

% Extract yaw angles (psi) from the state vector
% z(:,3)  = psi1  (segment 1)
% z(:,9)  = psi2  (segment 2)
% z(:,15) = psi3  (segment 3)
% z(:,21) = psiD  (drogue)

psi1_final = rad2deg(z(end,3));
psi2_final = rad2deg(z(end,9));
psi3_final = rad2deg(z(end,15));
psiD_final = rad2deg(z(end,21));

fprintf('\n========== FINAL ANGLES (last frame) ==========\n');
fprintf('Angle convention: 0° = horizontal right (positive x), positive = CCW\n');
fprintf('  S1 (psi1)      : %.2f deg\n', psi1_final);
fprintf('  S2 (psi2)      : %.2f deg\n', psi2_final);
fprintf('  S3 (psi3)      : %.2f deg\n', psi3_final);
fprintf('  Drogue (psiD)  : %.2f deg\n', psiD_final);
fprintf('==================================================\n\n');

%% ================= ANGULAR VELOCITY PLOTS =================

%% Segment 1
figure
plot(t, z(:,6),'LineWidth',1.8)
grid on
xlabel('Time (s)')
ylabel('\omega_{z,1} (rad/s)')
title('Segment 1 Angular Velocity')

%% Segment 2
figure
plot(t, z(:,12),'LineWidth',1.8)
grid on
xlabel('Time (s)')
ylabel('\omega_{z,2} (rad/s)')
title('Segment 2 Angular Velocity')

%% Segment 3
figure
plot(t, z(:,18),'LineWidth',1.8)
grid on
xlabel('Time (s)')
ylabel('\omega_{z,3} (rad/s)')
title('Segment 3 Angular Velocity')

%% Drogue
figure
plot(t, z(:,24),'LineWidth',2.2)
grid on
xlabel('Time (s)')
ylabel('\omega_{z,D} (rad/s)')
title('Drogue Angular Velocity')
%% ================= Hinge Angles PLOTS =================

%% Hinge 1
figure
plot(t, z(:,25), 'LineWidth', 1.8)
grid on
xlabel('Time (s)')
ylabel('\theta_{j1} (rad)')
title('Hinge 1 Angle')

%% Hinge 2
figure
plot(t, z(:,26), 'LineWidth', 1.8)
grid on
xlabel('Time (s)')
ylabel('\theta_{j2} (rad)')
title('Hinge 2 Angle')

%% Hinge 3
figure
plot(t, z(:,27), 'LineWidth', 1.8)
grid on
xlabel('Time (s)')
ylabel('\theta_{j3} (rad)')
title('Hinge 3 Angle')