function simulate_leg_squating()
clear all; close all; clc;

% ---------------------------------------------------------------------
% Setup robot parameters
% ---------------------------------------------------------------------
params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1;
params.w3 = 0.15;
params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.0;
params.g = 9.81;
params.I1 = (1/12)*params.m1*params.l1^2;
params.I2 = (1/12)*params.m2*params.l2^2;
params.I3 = (1/12)*params.m3*(params.w3^2 + params.l3^2);

% Virtual constraint gains
params.Kp = 50; params.Kd = 10;

% Trajectory Parameters
params.y_stand = 0.2;
params.y_squat  = 0.08;
params.y_takeoff = 0.255;
params.T_hold = 2.0;
params.T_squat = 1;
params.T_push  = 0.6;

% initial state x0 = [x; y; q1; q2; q3; xd; yd; q1d; q2d; q3d; u1; u2]
% Note: I keep the same state layout you used (you appended u placeholders into X)
x0 = [0; 0.2970; 0.2; 0.2; 0.0;  % positions
      0; 0; 0; 0; 0;             % velocities
      0; 0];                     % placeholders for control (u1,u2)

% global final time (fallback)
tf = 10;

% ---------------------------------------------------------------------
% PHASE A: Stance -> run stance dynamics until takeoff event triggers
% ---------------------------------------------------------------------
t_start = 0;
options_takeoff = odeset('Events', @(t,x) event_takeoff(t,x,params));

try
  [tA, XA, teA, xeA, ieA] = ode45(@(t,x) dynamics_stance(t,x,params), ...
                                  [t_start tf], x0, options_takeoff);
catch ME
  error('Error during stance integration (takeoff phase): %s', ME.message);
end

% If event triggered, get final state, else use last row
if ~isempty(teA)
    t_takeoff = teA(end);
    x_takeoff = xeA(end,:)';
else
    t_takeoff = tA(end);
    x_takeoff = XA(end,:)';
end

% ---------------------------------------------------------------------
% PHASE B: Flight -> integrate flight dynamics until touchdown event
% ---------------------------------------------------------------------
options_touchdown = odeset('Events', @(t,x) event_touchdown(t,x,params));

try
  [tB, XB, teB, xeB, ieB] = ode45(@(t,x) dynamics_flight(t,x,params), ...
                                  [t_takeoff tf], x_takeoff, options_touchdown);
catch ME
  error('Error during flight integration (touchdown phase): %s', ME.message);
end

if ~isempty(teB)
    t_touchdown = teB(end);
    x_touchdown = xeB(end,:)';
else
    t_touchdown = tB(end);
    x_touchdown = XB(end,:)';
end

% ---------------------------------------------------------------------
% PHASE C: Stance after touchdown -> run stance dynamics to final time
% ---------------------------------------------------------------------
% Continue simulation a bit after touchdown (or to tf)
try
  [tC, XC] = ode45(@(t,x) dynamics_stance(t,x,params), ...
                   [t_touchdown tf], x_touchdown);
catch ME
  error('Error during post-touchdown stance integration: %s', ME.message);
end

% ---------------------------------------------------------------------
% Concatenate results (preserve column count)
% ---------------------------------------------------------------------
% Make sure control columns exist for each segment (dynamics functions should append u into state)
T = [tA; tB; tC];
X = [XA; XB; XC];

% In case states had different column counts (shouldn't), assert consistent
if size(X,2) < 12
    % pad columns with zeros (safe-guard)
    X(:,size(X,2)+1:12) = 0;
end

% Plot results
plot_results(T, X, params);

end


function plot_results(t, X, params)
% X expected columns: 1..5 q, 6..10 dq, 11..12 u
figure;
X_sol = X(:,1:10);
u_sol = zeros(size(X,1),2);
if size(X,2) >= 12
    u_sol = X(:,11:12);
end

subplot(1,3,1); hold on; grid on;
plot(t, X_sol(:,3),'LineWidth',2,'DisplayName','q1'); % q1 is col 3
plot(t, X_sol(:,4),'LineWidth',2,'DisplayName','q2'); % q2
plot(t, X_sol(:,5),'LineWidth',2,'DisplayName','q3'); % q3
legend; title('Joint Angles (q1,q2,q3)'); xlabel('t (s)');

subplot(1,3,2); hold on; grid on;
plot(t, X_sol(:,8),'LineWidth',2,'DisplayName','dq1'); % dq1 is col 8 (6..10 => 6:xd,7:yd,8:q1d,9:q2d,10:q3d)
plot(t, X_sol(:,9),'LineWidth',2,'DisplayName','dq2');
plot(t, X_sol(:,10),'LineWidth',2,'DisplayName','dq3');
legend; title('Joint velocities (q1dot,q2dot,q3dot)'); xlabel('t (s)');

subplot(1,3,3); hold on; grid on;
plot(t, u_sol(:,1),'LineWidth',2,'DisplayName','u1');
plot(t, u_sol(:,2),'LineWidth',2,'DisplayName','u2');
legend; title('Control Inputs'); xlabel('t (s)');

end
