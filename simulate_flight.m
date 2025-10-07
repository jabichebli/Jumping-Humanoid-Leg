clear all; close all; clc;

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
    x0 = [-0.0133;0.2139;0.8509;1.5478;0; -0.0753; 0.98;-5.2010;-9.0183;0];         % initial velocities 

    tspan = [0, 20] ;
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


    % X_all = [X_sol; X_flight] ;
    % t_all = [t; t_flight] ;
    % 
    % figure(4); hold on;
    % subplot(1,3,1); hold on;
    % plot(t_all, X_all(:,3),'LineWidth',2, "DisplayName", "q1")
    % plot(t_all, X_all(:,4),'LineWidth',2, "DisplayName", "q2")
    % plot(t_all, X_all(:,5),'LineWidth',2, "DisplayName", "q3")
    % legend;
    % 
    % subplot(1,3,2); hold on;
    % plot(t_all, X_all(:,8),'LineWidth',2, "DisplayName", "dq1")
    % plot(t_all, X_all(:,9),'LineWidth',2, "DisplayName", "dq2")
    % plot(t_all, X_all(:,10),'LineWidth',2, "DisplayName", "dq3")
    % legend;
    % 
    % subplot(1,3,3); hold on;
    % plot(t_all, X_all(:,1),'LineWidth',2, "DisplayName", "x")
    % plot(t_all, X_all(:,2),'LineWidth',2, "DisplayName", "y")
    % legend;

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
    animate_leg(t_flight, X_flight, params);