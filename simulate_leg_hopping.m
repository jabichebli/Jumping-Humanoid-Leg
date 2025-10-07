function simulate_leg_hopping()

    clear all; close all; clc;
    
    % -------------------------------------------------------------------------
    % simulate_leg_hopping.m
    % -------------------------------------------------------------------------
    % Simulates one or multiple hops of the 3-link leg with stance/flight phases.
    % 
    % Phases:
    %   1. Stance phase  -> (event_takeoff)
    %   2. Flight phase  -> (event_touchdown)
    %   3. Impact map    -> reset velocities
    %
    % REQUIREMENTS:
    %   - dynamics_stance.m
    %   - dynamics_flight.m
    %   - impact_map.m
    %   - event_takeoff.m
    %   - event_touchdown.m
    %   - auto_* symbolic functions
    %   - animate_leg.m (for visualization)
    %
    % Author: Jason Abi Chebli & Andrew Tai
    % MECENG239 Final Project - Fall 2025
    % -------------------------------------------------------------------------
    
    % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.0; 
    params.g  = -9.81;
    
    % Moments of inertia
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);
    
    % Trajectory Parameters
    params.y_stand   = 0.2;
    params.y_squat   = 0.08;
    params.y_takeoff = 0.258;
    params.T_hold    = 3.0;
    params.T_squat   = 0.6;
    params.T_push    = 0.4;
    
    % Virtual constraint gains
    params.Kp = 80; 
    params.Kd = 10;

    % ---------------- Initial Conditions ----------------
    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0 = [0; 0.2970; 0.2; 0.2; 0.0;   % initial positions
          0; 0; 0; 0; 0;
          0 ; 0];         % initial velocities 

    t0 = 0;
    tf = 50;       % total simulation time
    t_all = [];
    X_all = [];

    % ---------------- Phase 1: Stance Phase ----------------
    disp('Starting stance phase...')
    [t_check, lambda_check] = dynamics_stance(0, [x0; zeros(3,1)], params);
    disp(['Initial Fy = ', num2str(lambda_check(2))]);

    takeoff_options = odeset('Events', @(t,x) event_takeoff(t,x,params));
    [t1, X1] = ode45(@(t,x) dynamics_stance(t,x,params), [t0 tf], x0, takeoff_options);
    disp('Takeoff detected!')

    t_all = [t_all; t1];
    X_all = [X_all; X1(:,1:10)];

    % Get takeoff state
    x_takeoff = X1(end,1:10)';

    % DEBUG
    % get stance final full state and diagnostics
    x_stance_end_full = X1(end, :);   % X1 has 13 columns from stance integrator
    % fetch last u and lambda & ddq by calling dynamics_stance once more:
    [dx_st, lambda_st] = dynamics_stance(t1(end), x_stance_end_full', params);
    u_st = dx_st(11:12);   % u returned inside dx (your dynamics_stance returns [dq;ddq;u])
    dq_st = x_stance_end_full(6:10);

    fprintf('AT TAKEOFF t=%.3f: Fy = %.6f, u = [%g %g], dq(end) = [%g %g %g %g %g]\n',...
        t1(end), lambda_st(2), u_st(1), u_st(2), dq_st);

    % ---------------- Phase 2: Flight Phase ----------------
    if t1(end) < tf
        disp('Starting flight phase...')
        touchdown_options = odeset('Events', @(t,x) event_touchdown(t,x,params));
        [t2, X2] = ode45(@(t,x) dynamics_flight(t,x,params), [t1(end) tf], x_takeoff, touchdown_options);
        disp('Touchdown detected!')

        % Append
        t_all = [t_all; t2];
        X_all = [X_all; X2(:,1:10)];

        % ---------------- Impact (Touchdown) ----------------
        x_preImpact = X2(end,1:10)';
        x_postImpact = impact_map(x_preImpact, params);

    else
        disp('Simulation ended before takeoff (no flight phase).')
    end

    % ---------------- Plot Results ----------------
    % figure(1); clf;
    % subplot(2,1,1); hold on;
    % plot(t_all, X_all(:,2), 'LineWidth', 2)
    % ylabel('Y (height)')
    % xlabel('Time [s]')
    % title('Center of Mass Height')
    % grid on
    % 
    % subplot(2,1,2); hold on;
    % plot(t_all, X_all(:,6), 'LineWidth', 2)
    % ylabel('Xdot (horizontal velocity)')
    % xlabel('Time [s]')
    % title('Horizontal Motion')
    % grid on

    % ---------------- Animate Result ----------------
    disp('Animating result...')
    animate_leg(t_all, X_all, params);

    disp(t_all)
    % disp(X_all)

    tt = linspace(0, params.T_hold + params.T_squat + params.T_push, 100);
    for i = 1:length(tt)
        y_path(i) = desired_COM_height(tt(i), params);
    end
    figure; plot(tt, y_path, 'LineWidth', 2); grid on;
    xlabel('Time (s)'); ylabel('Desired COM height');
    title('Planned COM Trajectory');


end
