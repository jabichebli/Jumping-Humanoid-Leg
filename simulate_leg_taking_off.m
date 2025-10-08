function simulate_leg_taking_off()

clear all; close all; clc;

    % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0; % The COM of torso would need to be aligned with hip 
    params.g  = 9.81; 

    % Moments of inertia
    params.I1 = (1/12) * params.m1 * params.l1^2; % thin rod approximation about center
    params.I2 = (1/12) * params.m2 * params.l2^2; % thin rod approximation about center
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2); % rectangular approximation about center

    % Virtual constraint gains
    params.Kp = 50; 
    params.Kd = 10;

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 0.5; 0.8; 1.2; 0; 0; 0; 0; 0; 0]; % flight
    
    % Start off the ground so we can land on the ground nicely
    tspan = [0, (2*3/9.81) + 2.5] ;
    opts = odeset('Events', @(t,x) event_touchdown(t,x,params), 'RelTol',1e-8,'AbsTol',1e-9);
    [t_flight, X_flight] = ode45(@(t,x) dynamics_flight(t,x,params), tspan, x0, opts);

    % The leg is now on the ground --> stabilize it
    x0_home= X_flight(end, :)';
    t0_home= t_flight(end);

    params.pCOMy_d = 0.2; % stabilize leg to pCOM_y = 0.20
    % Stabilize the leg
    [t_home, X_home] = ode45(@(t,x) dynamics_stance(t,x,params), [t0_home t0_home+3], x0_home);

    % % The leg is now on the ground
    % x0_stance = X_home(end, :)';
    % x0_stance(7) = 1.0;  % inject upward velocity (ydot = 1 m/s)
    % t0_stance = t_home(end);
    % 
    % opts_takeoff = odeset('Events', @(t,x) event_takeoff(t,x,params), 'RelTol',1e-8,'AbsTol',1e-10);
    % [t_stance, X_stance] = ode45(@(t,x) dynamics_stance(t,x,params), [t0_stance t0_stance+5], x0_stance, opts_takeoff);
    % 
    % t_all = [t_flight; t_home; t_stance];
    % X_all = [X_flight; X_home; X_stance];
    
    t_all = [t_flight; t_home];
    X_all = [X_flight; X_home];
    

    figure; hold on;
    subplot(2,2,1); hold on;
    plot(t_all, X_all(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_all, X_all(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_all, X_all(:,5),'LineWidth',2, "DisplayName", "q3")
    legend;

    subplot(2,2,2); hold on;
    plot(t_all, X_all(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_all, X_all(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_all, X_all(:,10),'LineWidth',2, "DisplayName", "dq3")

    subplot(2,2,3); hold on;
    plot(t_all, X_all(:,1),'LineWidth',2, "DisplayName", "x")
    plot(t_all, X_all(:,2),'LineWidth',2, "DisplayName", "y")
    legend;

    subplot(2,2,4); hold on;
    plot(t_all, X_all(:,6),'LineWidth',2, "DisplayName", "dx")
    plot(t_all, X_all(:,7),'LineWidth',2, "DisplayName", "dy")
    legend;

    % for i = 1:length(t_stance)
    %     [~,~,lambda,~] = dynamics_stance(t_stance(i), X_stance(i,:).', params);
    %     Fy(i) = -lambda(2);
    % end
    % figure; plot(t_stance, Fy); grid on;
    % xlabel('Time (s)'); ylabel('Ground Reaction Force Fy (N)');
    % title('Vertical Ground Reaction Force during Stance');

    % Animate result
    animate_leg(t_all, X_all, params);
end

