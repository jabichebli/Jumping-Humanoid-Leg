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
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0; % The COM of torso would need to be aligned with hip 
    params.g  = 9.81; 

    % Moments of inertia
    params.I1 = (1/12) * params.m1 * params.l1^2; % thin rod approximation about center
    params.I2 = (1/12) * params.m2 * params.l2^2; % thin rod approximation about center
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2); % rectangular approximation about center

    params.pCOMy_d = 0.2; % standing: 0.20 --> squatting: 0.08  --> takeoff: 0.258

    % Virtual constraint gains
    params.Kp = 50; 
    params.Kd = 10;

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 0.204353663171501; 1.2; 1.2; 0.0; 0; 0; 0; 0; 0; 0 ; 0;];

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
    % plot(t, u_sol(:,3),'LineWidth',2, "DisplayName", "u(q2)")
    legend;



    % Animate result
    animate_leg(t, X, params);
end


% --------------------- JASON ---------------------------------------

% function simulate_leg()
% % -------------------------------------------------------------------------
% % simulate_leg.m
% % -------------------------------------------------------------------------
% % Simulates a 3-link hopping leg through stance/flight/impact transitions.
% %
% % REQUIREMENTS:
% %   - dynamics_stance_with_lambda.m
% %   - dynamics_flight.m
% %   - event_takeoff.m
% %   - event_touchdown.m
% %   - impact_map.m
% %   - animate_leg.m
% % -------------------------------------------------------------------------
% 
%     % ---------------- Simulation Parameters ----------------
%     params.m1 = 5;   params.m2 = 3;   params.m3 = 7;
%     params.I1 = 0.1; params.I2 = 0.05; params.I3 = 0.2;
%     params.l1 = 0.5; params.l2 = 0.5; params.l3 = 0.2;
%     params.d1 = 0.25; params.d2 = 0.25; params.d3 = 0.3;
%     params.g  = 9.81;
% 
%     % Virtual constraint gains
%     params.Kp = 100; 
%     params.Kd = 20;
% 
%     % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
%     x0 = [0; 1; 0.1; -0.2; 0.0; 0; 0; 0; 0; 0];
% 
%     % Final simulation time
%     tFinal = 6.0;
% 
%     % Run hybrid simulation
%     [T, X, PhaseLog] = hybrid_leg_sim(x0, params, tFinal);
% 
%     % Animate result
%     animate_leg(T, X, params);
% end
% 
% % -------------------------------------------------------------------------
% function [T, X, PhaseLog] = hybrid_leg_sim(x0, params, tFinal)
%     T = []; X = []; PhaseLog = [];
%     t0 = 0; x = x0;
%     phase = "stance";  % assume starting on ground
% 
%     while t0 < tFinal
%         switch phase
%             case "stance"
%                 opts = odeset('RelTol',1e-4,'AbsTol',1e-6, 'MaxStep', 0.05, 'Events', @(t,y) event_takeoff(t,y,params));
%                 [t, y] = ode15s(@(t,y) stance_wrapper(t,y,params), [t0 tFinal], x, opts);
% 
%                 T = [T; t]; X = [X; y]; 
%                 PhaseLog = [PhaseLog; repmat("stance", length(t), 1)];
% 
%                 if t(end) < tFinal
%                     % Takeoff happened
%                     x = y(end,:)';
%                     phase = "flight";
%                 else
%                     break;
%                 end
% 
%             case "flight"
%                 opts = odeset('RelTol',1e-4,'AbsTol',1e-6, 'MaxStep', 0.05, 'Events', @(t,y) event_touchdown(t,y,params));
%                 [t, y] = ode15s(@(t,y) dynamics_flight(t,y,params), [t0 tFinal], x, opts);
% 
%                 T = [T; t]; X = [X; y];
%                 PhaseLog = [PhaseLog; repmat("flight", length(t), 1)];
% 
%                 if t(end) < tFinal
%                     % Touchdown -> apply impact map
%                     x_pre = y(end,:)';
%                     x_post = impact_map(x_pre, params);
%                     x = x_post;
%                     phase = "stance";
%                 else
%                     break;
%                 end
%         end
%         t0 = T(end);
%     end
% end
% 
% % -------------------------------------------------------------------------
% 
% function dx = stance_wrapper(t, x, params)
%     [dx, ~] = dynamics_stance(t, x, params); % call with t,x,params
% end
% 
% 
% % % ---------- Inverse Kinematics Problem (with COM Constraint) ------------- 
% % % Assume the foot is at (0,0) in a world frame
% % eq1 = (x_hip + l1*sin(q1) + l2*sin(q1 + q2)) == 0; %xf = 0 
% % eq2 = (y_hip + l1*cos(q1) + l2*cos(q1 + q2)) == 0; %yf = 0 
% % eq3 = (m1*(x_hip + d1*sin(q1)) + m2*(x_hip + l1*sin(q1) + d2*sin(q1 + q2))) / (m1 + m2) == 0; %pCOM_x = 0
% % 
% % % Solve the system for the unknowns x_hip, q1, and q2
% % solutions = solve([eq1, eq2, eq3], [x_hip, q1, q2], 'Real', true);
% % x_hip_sym = solutions.x_hip;
% % q1_sym = solutions.q1;
% % q2_sym = solutions.q2;
% % this 
% % % Convert the symbolic solution to a fast MATLAB function
% % inv_kin_func = matlabFunction([x_hip_sym(1), q1_sym(1), q2_sym(1)], 'Vars', {y_hip, l1, l2, d1, d2, m1, m2});