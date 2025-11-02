% test_trajectory.m
clear all; close all; clc;

% --- Define parameters for the test ---
params.y_stand_target = 0.22; % The desired height for the initial balanced pose
params.y_stand = params.y_stand_target;
params.T_jump         = 0.7;  % Total time for the jump motion (dip + push)
params.y_squat        = 0.06; % The deepest the CoM is allowed to go.
params.y_takeoff      = 0.26;
params.T_hold         = 1.0;
params.vf_takeoff     = 0.5;
params.T_dip_ratio  = 0.75;  % <-- You can optionally uncomment and tune this!

params.Kp = 200; params.Kd = 20;

% --- Generate the trajectory data ---
t_end = params.T_hold + params.T_jump;
t_step = 0.001; % Use a smaller time step for smoother plots
t = 0:t_step:t_end;
p_d = zeros(size(t));
v_d = zeros(size(t));
a_d = zeros(size(t));

for i = 1:length(t)
    [p_d(i), v_d(i), a_d(i)] = desired_COMy(t(i), params);
end

% --- Plot the results ---
figure('Name', 'Smooth Jump Trajectory Analysis');

% Define phase times for plotting
t_hold_end = params.T_hold;
if isfield(params, 'T_dip_ratio')
    t_dip_ratio = params.T_dip_ratio;
else
    t_dip_ratio = 0.4;
end
t_dip_end = params.T_hold + params.T_jump * t_dip_ratio;
t_jump_end = params.T_hold + params.T_jump;

% Position Plot
subplot(3,1,1);
plot(t, p_d, 'b-', 'LineWidth', 2);
hold on;
% Plot vertical lines for phase changes
xline(t_hold_end, 'k--', 'Hold End');
xline(t_dip_end, 'k--', 'Squat Bottom');
xline(t_jump_end, 'k--', 'Takeoff');
grid on;
title('Desired COM Position ($y_d$)', 'Interpreter', 'latex');
ylabel('Position (m)');
legend('y_d', 'Location', 'southeast');

% Velocity Plot
subplot(3,1,2);
plot(t, v_d, 'r-', 'LineWidth', 2);
hold on;
xline(t_hold_end, 'k--');
xline(t_dip_end, 'k--');
xline(t_jump_end, 'k--');
grid on;
title('Desired COM Velocity ($\dot{y}_d$)', 'Interpreter', 'latex');
ylabel('Velocity (m/s)');

% Acceleration Plot
subplot(3,1,3);
plot(t, a_d, 'g-', 'LineWidth', 2);
hold on;
xline(t_hold_end, 'k--');
xline(t_dip_end, 'k--');
xline(t_jump_end, 'k--');
grid on;
title('Desired COM Acceleration ($\ddot{y}_d$)', 'Interpreter', 'latex');
ylabel('Accel (m/s^2)');
xlabel('Time (s)');
% 
% % test_trajectory.m
% clear all; close all; clc;
% 
% % --- Define parameters for the test ---
% params.y_stand_target = 0.22; % The desired height for the initial balanced pose
% params.y_stand = params.y_stand_target;
% 
% params.T_jump         = 0.7;  % Total time for the jump motion (dip + push)
% params.y_squat        = 0.04; % The deepest the CoM is allowed to go.
% params.y_takeoff      = 0.26;
% params.T_hold         = 1.0;
% params.vf_takeoff     = 1.8;
% 
% 
% % --- Generate the trajectory data ---
% t_end = params.T_hold + params.T_jump + 0.5;
% %t_end = params.T_hold + params.T_squat + params.T_push + 0.5;
% 
% t = 0:0.01:t_end;
% p_d = zeros(size(t));
% v_d = zeros(size(t));
% a_d = zeros(size(t));
% 
% for i = 1:length(t)
%     [p_d(i), v_d(i), a_d(i)] = desired_COMy(t(i), params);
% end
% 
% % --- Plot the results ---
% figure('Name', 'Quintic Polynomial Trajectory Analysis');
% 
% % Position Plot
% subplot(3,1,1);
% plot(t, p_d, 'b-', 'LineWidth', 2);
% grid on;
% title('Desired COM Position ($y_d$)', 'Interpreter', 'latex');
% ylabel('Position (m)');
% 
% % Velocity Plot
% subplot(3,1,2);
% plot(t, v_d, 'r-', 'LineWidth', 2);
% grid on;
% title('Desired COM Velocity ($\dot{y}_d$)', 'Interpreter', 'latex');
% ylabel('Velocity (m/s)');
% 
% % Acceleration Plot
% subplot(3,1,3);
% plot(t, a_d, 'g-', 'LineWidth', 2);
% grid on;
% title('Desired COM Acceleration ($\ddot{y}_d$)', 'Interpreter', 'latex');
% ylabel('Accel (m/s^2)');
% xlabel('Time (s)');