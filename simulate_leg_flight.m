function simulate_leg_flight()


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

    % Virtual constraint gains
    params.Kp = 50; 
    params.Kd = 10;


    % params.pCOMy_d = 0.2; % standing: 0.20 --> squatting: 0.08  --> takeoff: 0.258

    params.m_t = params.m1 + params.m2 + params.m3 ;

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 0.26; 0.8; 1.2; 0; 0; 3; 0; 0; 0];

    tspan = [0, (2*3/9.81) + 2.5] ;
    opts = odeset('Events', @(t,x) event_touchdown(t,x,params));
    [t_flight, X_flight] = ode45(@(t,x) dynamics_flight(t,x,params), tspan, x0, opts);

    figure(5); hold on;
    subplot(2,2,1); hold on;
    plot(t_flight, X_flight(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_flight, X_flight(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_flight, X_flight(:,5),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(2,2,2); hold on;
    plot(t_flight, X_flight(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_flight, X_flight(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_flight, X_flight(:,10),'LineWidth',2, "DisplayName", "dq3")

    subplot(2,2,3); hold on;
    plot(t_flight, X_flight(:,1),'LineWidth',2, "DisplayName", "x")
    plot(t_flight, X_flight(:,2),'LineWidth',2, "DisplayName", "y")
    legend;

    subplot(2,2,4); hold on;
    plot(t_flight, X_flight(:,6),'LineWidth',2, "DisplayName", "dx")
    plot(t_flight, X_flight(:,7),'LineWidth',2, "DisplayName", "dy")
    legend;
    
    % Animate leg falling
    animate_leg(t_flight, X_flight, params);

end