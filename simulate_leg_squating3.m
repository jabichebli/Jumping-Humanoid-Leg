function simulate_leg_squating()
clear all; close all; clc;

%% ---------------- Robot & control params ----------------
params.m1 = 0.15; params.m2 = 0.15; params.m3 = 0.8;
params.l1 = 0.15; params.l2 = 0.15; params.l3 = 0.1;
params.w3 = 0.15;
params.d1 = 0.075; params.d2 = 0.075; params.d3 = 0.0;
params.g = 9.81;
params.I1 = (1/12)*params.m1*params.l1^2;
params.I2 = (1/12)*params.m2*params.l2^2;
params.I3 = (1/12)*params.m3*(params.w3^2 + params.l3^2);
params.Kp = 50; params.Kd = 10;
params.alpha = 0.1; % smoothing factor for control

% Trajectory Parameters
params.y_stand = 0.2;
params.y_squat  = 0.08;
params.y_takeoff = 0.255;
params.T_hold = 2.0;
params.T_squat = 1;
params.T_push  = 0.6;

%% ---------------- Initial state ----------------
x = [0;0.297;0.2;0.2;0; 0;0;0;0;0]; % [q; dq]
u_prev = [0;0];                     % initial control
tf = 10;
t_start = 0;

T = []; X = []; U = [];

t = t_start;

%% ---------------- PHASE A: Stance ----------------
while t < tf
    t_span = [t, t+0.01]; % small step
    [t_step, x_step] = ode45(@(t,x) dynamics_stance(t,x,u_prev,params), t_span, x);

    % Store results
    T = [T; t_step];
    X = [X; x_step];

    % Update control using last state
    [~, ~, u_out] = dynamics_stance(t_step(end), x_step(end,:)', u_prev, params);
    U = [U; repmat(u_out', length(t_step), 1)];
    u_prev = u_out;

    % Advance state
    t = t_step(end);
    x = x_step(end,:)';

    % Check takeoff event
    if event_takeoff(t,x,params)
        fprintf('Takeoff detected at t = %.3f\n', t);
        break;
    end
end

x_takeoff = x;
u_takeoff = u_prev;

%% ---------------- PHASE B: Flight ----------------
while t < tf
    t_span = [t, t+0.01];
    [t_step, x_step] = ode45(@(t,x) dynamics_flight(t,x,u_prev,params), t_span, x);

    % Store results
    T = [T; t_step];
    X = [X; x_step];

    % Flight control held constant
    U = [U; repmat(u_prev', length(t_step), 1)];

    % Advance state
    t = t_step(end);
    x = x_step(end,:)';

    % Check touchdown
    if event_touchdown(t,x,params)
        fprintf('Touchdown detected at t = %.3f\n', t);
        break;
    end
end

x_touchdown = x;

%% ---------------- PHASE C: Post-touchdown stance ----------------
while t < tf
    t_span = [t, t+0.01];
    [t_step, x_step] = ode45(@(t,x) dynamics_stance(t,x,u_prev,params), t_span, x);

    % Store results
    T = [T; t_step];
    X = [X; x_step];

    % Update control using last state
    [~, ~, u_out] = dynamics_stance(t_step(end), x_step(end,:)', u_prev, params);
    U = [U; repmat(u_out', length(t_step), 1)];
    u_prev = u_out;

    % Advance state
    t = t_step(end);
    x = x_step(end,:)';
end

%% ---------------- Plot results ----------------
plot_results(T, X, U, params);

end

%% ---------------- Plot function ----------------
function plot_results(T, X, U, params)
figure;

subplot(1,3,1); hold on; grid on;
plot(T, X(:,3:5),'LineWidth',2); legend('q1','q2','q3'); title('Joint Angles'); xlabel('t (s)');

subplot(1,3,2); hold on; grid on;
plot(T, X(:,8:10),'LineWidth',2); legend('dq1','dq2','dq3'); title('Joint Velocities'); xlabel('t (s)');

subplot(1,3,3); hold on; grid on;
plot(T, U(:,1),'LineWidth',2); plot(T, U(:,2),'LineWidth',2);
legend('u1','u2'); title('Control Inputs'); xlabel('t (s)');
end
