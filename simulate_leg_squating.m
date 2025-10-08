function simulate_leg_squating()

clear all; close all; clc;
% -------------------------------------------------------------------------
% simulate_leg.m
% -------------------------------------------------------------------------
% Simulates the dynamics of a 3-link hopping leg with holonomic constraints
% and virtual constraint control. 
%
% REQUIREMENTS:
%   - Symbolic functions auto_D, auto_C, auto_G, auto_B
%   - Symbolic functions auto_h, auto_Jh, auto_d2h__
%   - Foot position function auto_pfoot
%   - ODE solver (ode45)
%
% OUTPUT:
%   - Time series of states (q, dq)
%   - Calls animate_leg() to visualize results
%
% Author: Jason Abi Chebli & Andrew Tai
% MECENG239 Final Project - Fall 2025
% -------------------------------------------------------------------------

    % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0; % The COM of torso would need to be aligned with hip 
    params.g  = 9.81; 
    params.q1d = 0; params.q2d = 0;

    % Moments of inertia
    params.I1 = (1/12) * params.m1 * params.l1^2; % thin rod approximation about center
    params.I2 = (1/12) * params.m2 * params.l2^2; % thin rod approximation about center
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2); % rectangular approximation about center

    % params.pCOMy_d = 0.2; % standing: 0.20 --> squatting: 0.08  --> takeoff: 0.258

    params.m_t = params.m1 + params.m2 + params.m3 ;
    v_takeoff = sqrt(2*params.g*0.10) ;


    % Trajectory Parameters - Balanced for proper takeoff
    params.y_stand   = 0.2;
    params.y_squat   = 0.08;    % Moderate squat depth
    params.y_takeoff = 0.5;    % Reasonable takeoff height
    params.T_hold    = 2.0;
    params.T_squat   = 0.6;
    params.T_push    = 0.3;     % Moderate push phase

    % Virtual constraint gains - Conservative control
    params.Kp = 30;   % Lower position gain for stability
    params.Kd = 8;    % Lower velocity gain for stability

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 0.2970; 0.2; 0.2; 0.0;   % initial positions
          0; 0; 0; 0; 0;];         % initial velocities 


    % Time span
    tspan = [0 10];

    % ODE solve

    % More robust ODE solver settings for aggressive dynamics
    opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01);
    [t, X] = ode45(@(t,x) dynamics_stance(t,x,params), tspan, x0, opts);
    disp(length(t))

    u_sol = zeros(length(t), 2);
    lambda_sol = zeros(length(t), 2);
    yCOM_sol = zeros(length(t), 1);



    for i = 1:length(t)
        [~, u, lambda, yCOM] = dynamics_stance(t(i), X(i,:).', params);
        u_sol(i,:) = u.';
        lambda_sol(i,:) = lambda.';
        yCOM_sol(i,:) = yCOM.';
    end

    X_sol = X(:, 1:10) ;

    % dyCOM_sol = diff(yCOM_sol)./diff(t) ;
    % % takeoff_idx = find(dyCOM_sol > v_takeoff) ;
    % % takeoff_idx = find(dyCOM_sol > 2.5) ;
    % 
    % figure(3); hold on;
    % % plot(t, lambda_sol(:,1),'LineWidth',2, "DisplayName", "lambda(x)")
    % plot(t, yCOM_sol,'LineWidth',2, "DisplayName", "yCOM_sol")
    % plot(t(1:end-1), dyCOM_sol,'LineWidth',2, "DisplayName", "dyCOM_sol")
    % 
    % % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    % legend;
    % disp(yCOM_sol(takeoff_idx(end)))


    % Takeoff when COM reaches the target height
    takeoff_idx = find(yCOM_sol > params.y_takeoff) ;
    if isempty(takeoff_idx)
        % If never reaches target, take off at end
        single_takeoff_idx = length(t);
        disp("Not enough speed to take off")
    else
        single_takeoff_idx = takeoff_idx(1);
    end

    t = t(1:single_takeoff_idx, :) ;
    X_sol = X_sol(1:single_takeoff_idx, :) ;
    u_sol = u_sol(1:single_takeoff_idx, :) ;
    lambda_sol = lambda_sol(1:single_takeoff_idx, :) ;

    disp('Final stance state:');
    disp(X_sol(end, :)) ;
    
    % Calculate and display takeoff velocity
    ydot_takeoff = X_sol(end, 7);  % y velocity at takeoff
    fprintf('Takeoff y-velocity: %.3f m/s\n', ydot_takeoff);

    figure(1); hold on;
    subplot(3,2,1); hold on;
    plot(t, X_sol(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t, X_sol(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t, X_sol(:,5),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(3,2,2); hold on;
    plot(t, X_sol(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t, X_sol(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t, X_sol(:,10),'LineWidth',2, "DisplayName", "dq3")

    subplot(3,2,3); hold on;
    plot(t, u_sol(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t, u_sol(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    legend;

    subplot(3,2,4); hold on;
    % plot(t, lambda_sol(:,1),'LineWidth',2, "DisplayName", "lambda(x)")
    plot(t, lambda_sol(:,2),'LineWidth',2, "DisplayName", "lambda(y)")
    % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    legend;

    
    disp(t(end))



    t = t(2:end, :) ;
    X_sol = X_sol(2:end, :) ;
    u_sol = u_sol(2:end, :) ;
    lambda_sol = lambda_sol(2:end, :) ;
    x0 = X_sol(end, :);
    disp('Final stance state (should have velocities):');
    disp(x0)
    
    % Verify we have the full state vector with velocities
    if length(x0) ~= 10
        error('State vector should have 10 elements: [x,y,q1,q2,q3,xdot,ydot,q1dot,q2dot,q3dot]');
    end
    
    % Display the velocities specifically
    fprintf('x0 velocities: [xdot=%.3f, ydot=%.3f, q1dot=%.3f, q2dot=%.3f, q3dot=%.3f]\n', ...
            x0(6), x0(7), x0(8), x0(9), x0(10));

    params.q1d = x0(3); params.q2d = x0(4);

    % Don't zero out velocities - preserve them from stance phase
    % x0(8:10) = 0;  % REMOVED: This was causing the problem!

    tspan = [t(end), t(end)+1] ;
    % opts = odeset('Events', @(t,x) event_touchdown(t,x,params));
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
    [t_flight, X_flight] = ode45(@(t,x) dynamics_flight(t,x,params), tspan, x0, options);
    disp('Flight phase started with initial state:');
    disp(X_flight(1, :));
    fprintf('Initial flight y-velocity: %.3f m/s\n', X_flight(1, 7));

    figure(5); hold on;
    subplot(1,2,1); hold on;
    plot(t_flight, X_flight(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_flight, X_flight(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_flight, X_flight(:,5),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(1,2,2); hold on;
    plot(t_flight, X_flight(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_flight, X_flight(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_flight, X_flight(:,10),'LineWidth',2, "DisplayName", "dq3")


    X_all = [X_sol; X_flight] ;
    t_all = [t; t_flight] ;

    figure(4); hold on;
    subplot(1,3,1); hold on;
    plot(t_all, X_all(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_all, X_all(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_all, X_all(:,5),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(1,3,2); hold on;
    plot(t_all, X_all(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_all, X_all(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_all, X_all(:,10),'LineWidth',2, "DisplayName", "dq3")
    legend;

    subplot(1,3,3); hold on;
    plot(t_all, X_all(:,1),'LineWidth',2, "DisplayName", "x")
    plot(t_all, X_all(:,2),'LineWidth',2, "DisplayName", "y")
    legend;

    % subplot(3,2,3); hold on;
    % plot(t, u_sol(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    % plot(t, u_sol(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    % % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    % legend;

    % subplot(3,2,4); hold on;
    % % plot(t, lambda_sol(:,1),'LineWidth',2, "DisplayName", "lambda(x)")
    % plot(t, lambda_sol(:,2),'LineWidth',2, "DisplayName", "lambda(y)")
    % % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    % legend;
    animate_leg(t_all, X_all, params);


end