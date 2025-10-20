function simulate_leg_balance()

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
    params = getParams();

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 0.2970; 0.2; 0.2; 0.0;   % initial positions
          0; 0; 0; 0; 0;];         % initial velocities 

    % Time span
    tspan = [0 10];

    % ODE solve

    opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
    [t, X] = ode45(@(t,x) leg_ode(t,x,params), tspan, x0, opts);
    disp(length(t))

    u_sol = zeros(length(t), 2);
    for i = 1:length(t)
        [~, u] = leg_ode(t(i), X(i,:).', params);
        u_sol(i,:) = u.';
    end

    X_sol = X(:, 1:10) ;
    % u_sol = X(:, 11:12) ;
    disp(X)


    figure(1); hold on;
    subplot(1,3,1); hold on;
    plot(t, X_sol(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t, X_sol(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t, X_sol(:,5),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(1,3,2); hold on;
    plot(t, X_sol(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t, X_sol(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t, X_sol(:,10),'LineWidth',2, "DisplayName", "dq3")

    subplot(1,3,3); hold on;
    plot(t, u_sol(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t, u_sol(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    legend;

    % Animate result
    animate_leg(t, X, params);
end

