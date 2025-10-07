function simulate_leg_squating()
    clear all; close all; clc;

    % ---------------------------------------------------------------------
    %  Setup robot parameters
    % ---------------------------------------------------------------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.0; 
    params.g  = 9.81; 

    params.I1 = (1/12)*params.m1*params.l1^2;
    params.I2 = (1/12)*params.m2*params.l2^2;
    params.I3 = (1/12)*params.m3*(params.w3^2 + params.l3^2);

    % Virtual constraint gains
    params.Kp = 50; 
    params.Kd = 10;

    % Trajectory Parameters
    params.y_stand   = 0.2;
    params.y_squat   = 0.08;
    params.y_takeoff = 0.258;
    params.T_hold    = 2.0;
    params.T_squat   = 0.6;
    params.T_push    = 0.4;

    % ---------------------------------------------------------------------
    %  Define sequence of desired COM positions
    % ---------------------------------------------------------------------
    pCOMy_home = 0.2;

    % Initial state
    x0 = [0; 0.2970; 0.2; 0.2; 0.0;   % positions
          0; 0; 0; 0; 0;              % velocities
          0; 0];                      % control placeholders

    % Time span
    tf = 10;

    % Get leg from inital stance --> home stance 
    [t0, X0] = ode45(@(t,x) leg_ode(t,x,params), tspan, x0);
    x0 = X0(end, :)'; % Update initial condition for next phase

    % Make leg squat and lift
    takeoff_options = odeset('Events', @(t,x) event_takeoff(t,x,params));
    [t1, X1] = ode45(@(t,x) dynamics_stance(t,x,params), [t0(end) tf], x0, takeoff_options);
    x1 = X1(end, :)'; % Update initial condition for next phase

    % Leg is in flight
    touchdown_options = odeset('Events', @(t,x) event_touchdown(t,x,params));
    [t2, X2] = ode45(@(t,x) dynamics_flight(t,x,params), [t1(end) tf], x1, touchdown_options);
    x2 = X2(end, :)'; % Update initial condition for next phase

    % Get leg from touch-down stance --> home stance
    [t3, X3] = ode45(@(t,x) leg_ode(t,x,params), [t2(end) tf], x2);


    % Store results for plotting
    t = [t0; t1; t2; t3];
    X = [X0; X1; X2; X3];

    % Plot the results
    plot_results(t, X, params);



function plot_results(t, X, params)
    X_sol = X(:, 1:10);
    u_sol = X(:, 11:12);

    subplot(1,3,1); hold on;
    plot(t, X_sol(:,1),'LineWidth',2, "DisplayName", "q1")
    plot(t, X_sol(:,2),'LineWidth',2, "DisplayName", "q2")
    plot(t, X_sol(:,3),'LineWidth',2, "DisplayName", "q3")
    legend; title('Joint Angles');

    subplot(1,3,2); hold on;
    plot(t, X_sol(:,6),'LineWidth',2, "DisplayName", "dq1")
    plot(t, X_sol(:,7),'LineWidth',2, "DisplayName", "dq2")
    plot(t, X_sol(:,8),'LineWidth',2, "DisplayName", "dq3")
    title('Joint Velocities');

    subplot(1,3,3); hold on;
    plot(t, u_sol(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t, u_sol(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    legend; title('Control Inputs');
end
