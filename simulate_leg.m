function simulate_leg()

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
    params.m1 = 5;   params.m2 = 3;   params.m3 = 7;
    params.I1 = 0.1; params.I2 = 0.05; params.I3 = 0.2;
    params.l1 = 0.5; params.l2 = 0.5; params.l3 = 0.2;
    params.d1 = 0.25; params.d2 = 0.25; params.d3 = 0.3;
    params.g  = 9.81;

    % Virtual constraint gains
    params.Kp = 0.5; 
    params.Kd = 0.2;

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 1; 0.1; -0.2; 0.0;   % initial positions
          0; 0; 0; 0; 0;
          0 ; 0];         % initial velocities

    % Time span
    tspan = [0 3];

    % ODE solve

    opts = odeset('RelTol',1e-10,'AbsTol',1e-10);
    [t, X] = ode45(@(t,x) leg_ode(t,x,params), tspan, x0, opts);
    disp(length(t))

    X_sol = X(:, 1:10) ;
    u_sol = X(:, 11:12) ;


    figure(1); hold on;
    subplot(1,3,1); hold on;
    plot(t, X_sol(:,1),'LineWidth',2, "DisplayName", "q1")
    plot(t, X_sol(:,2),'LineWidth',2, "DisplayName", "q2")
    plot(t, X_sol(:,3),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(1,3,2); hold on;
    plot(t, X_sol(:,6),'LineWidth',2, "DisplayName", "dq1")
    plot(t, X_sol(:,7),'LineWidth',2, "DisplayName", "dq2")
    plot(t, X_sol(:,8),'LineWidth',2, "DisplayName", "dq3")

    subplot(1,3,3); hold on;
    plot(t, u_sol(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t, u_sol(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    legend;



    % Animate result
    % animate_leg(t, X, params, 3);
end


% % ---------------------------- During Impact ------------------------------
% function [dq_plus, Lambda] = impact_dynamics(q, dq_minus, D, Jst)
%     % Inputs:
%     %   q: Generalized coordinates
%     %   dq_minus: Pre-impact generalized velocities
%     %   D: Inertia matrix
%     %   Jst: Stance Jacobian (partial_pfoot/partial_q) at impact
%     %
%     % Outputs:
%     %   dq_plus: Post-impact generalized velocities
%     %   Lambda: Contact impulse at impact
% 
%     % Assemble the block system
%     A = [D, -Jst.'; 
%          Jst, zeros(size(Jst,1))];
% 
%     % Right-hand side
%     b = [D*dq_minus; 
%          zeros(size(Jst,1),1)];
% 
%     % Solve for [dq_plus; Lambda]
%     sol = A \ b;
%     dq_plus = simplify(sol(1:length(q)));
%     Lambda  = simplify(sol(length(q)+1:end));
% end


% % ---------- Inverse Kinematics Problem (with COM Constraint) ------------- 
% % Assume the foot is at (0,0) in a world frame
% eq1 = (x_hip + l1*sin(q1) + l2*sin(q1 + q2)) == 0; %xf = 0 
% eq2 = (y_hip + l1*cos(q1) + l2*cos(q1 + q2)) == 0; %yf = 0 
% eq3 = (m1*(x_hip + d1*sin(q1)) + m2*(x_hip + l1*sin(q1) + d2*sin(q1 + q2))) / (m1 + m2) == 0; %pCOM_x = 0
% 
% % Solve the system for the unknowns x_hip, q1, and q2
% solutions = solve([eq1, eq2, eq3], [x_hip, q1, q2], 'Real', true);
% x_hip_sym = solutions.x_hip;
% q1_sym = solutions.q1;
% q2_sym = solutions.q2;
% 
% % Convert the symbolic solution to a fast MATLAB function
% inv_kin_func = matlabFunction([x_hip_sym(1), q1_sym(1), q2_sym(1)], 'Vars', {y_hip, l1, l2, d1, d2, m1, m2});