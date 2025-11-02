function simulate_leg_squating()

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

    params.m_t = params.m1 + params.m2 + params.m3 ;
    % v_takeoff = sqrt(2*params.g*0.10) ;

    % Trajectory Parameters (Be careful tuning these)
    params.y_stand   = 0.2;
    params.y_squat   = 0.08;
    params.y_takeoff = 0.65;    % Takeoff height to produce enough y-velocity (actual takeoff height: ~0.258m)
    params.T_hold    = 2.0;
    params.T_squat   = 0.6;
    params.T_push    = 0.3;

    % Virtual constraint gains - Conservative control
    params.Kp = 30;
    params.Kd = 8;

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot]
    x0_stance = [0; 0.2970; 0.2; 0.2; 0;
          0;      0;   0;   0; 0;];

    % ------------------- Stabilize and then push off ------------------- 

    % Time span
    tspan_stance = [0 10];

    % Solve ODE45 during stance
    stance_options = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01);
    [t, X] = ode45(@(t,x) dynamics_stance(t,x,params), tspan_stance, x0_stance, stance_options);

    % Initialize parameters to extract variables during stance phase
    u_stance = zeros(length(t), 2);
    lambda_stance = zeros(length(t), 2);
    yCOM_stance = zeros(length(t), 1);

    % Extract controller input, lambda and COM during stance phase
    for i = 1:length(t)
        [~, u, lambda, yCOM] = dynamics_stance(t(i), X(i,:).', params);
        u_stance(i,:) = u.';
        lambda_stance(i,:) = lambda.';
        yCOM_stance(i,:) = yCOM.';
    end
    
    % Takeoff when COM reaches the target height
    takeoff_idx = find(yCOM_stance > params.y_takeoff) ;
    if isempty(takeoff_idx)
        % If never reaches target, take off at end
        single_takeoff_idx = length(t);
        disp("Not enough speed to take off")
    else
        single_takeoff_idx = takeoff_idx(1);
    end
    
    % Extract the stance time, position, controller input and force
    t_stance = t(1:single_takeoff_idx, :) ;
    X_stance = X(1:single_takeoff_idx, :) ;
    u_stance = u_stance(1:single_takeoff_idx, :) ;
    lambda_stance = lambda_stance(1:single_takeoff_idx, :) ;

    % Debug Statement
    disp('[Debug] Final stance state:');
    disp(X_stance(end, :)) ;
    
    % Debug Statement
    ydot_takeoff = X_stance(end, 7);  % y-velocity at takeoff
    fprintf('[Debug] Takeoff y-velocity: %.3f m/s\n', ydot_takeoff);

    % Plot position, angles, controller input, force for stance
    figure; 
    sgtitle("Stance -> Takeoff Phase", 'FontSize', 16, 'FontWeight', 'bold');

    subplot(3,2,1); hold on;
    plot(t_stance, X_stance(:,1),'LineWidth',2, "DisplayName", "x")
    plot(t_stance, X_stance(:,2),'LineWidth',2, "DisplayName", "y")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Position (m)");

    subplot(3,2,2); hold on;
    plot(t_stance, X_stance(:,6),'LineWidth',2, "DisplayName", "dx")
    plot(t_stance, X_stance(:,7),'LineWidth',2, "DisplayName", "dy")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Velocity (m/s)");

    subplot(3,2,3); hold on;
    plot(t_stance, X_stance(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_stance, X_stance(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_stance, X_stance(:,5),'LineWidth',2, "DisplayName", "q3")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Angle (rad)");

    subplot(3,2,4); hold on;
    plot(t_stance, X_stance(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_stance, X_stance(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_stance, X_stance(:,10),'LineWidth',2, "DisplayName", "dq3")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Angular Velocity (rad/s)");

    subplot(3,2,5); hold on;
    plot(t_stance, u_stance(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t_stance, u_stance(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Controller Input (N m)");

    subplot(3,2,6); hold on;
    plot(t_stance, lambda_stance(:,1),'LineWidth',2, "DisplayName", "lambda(x)")
    plot(t_stance, lambda_stance(:,2),'LineWidth',2, "DisplayName", "lambda(y)")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Ground Reaction Force (N)");

    % --------------------------- Flight Phase ----------------------------
    
    % Setup the initial conditions for flight using final stance state
    t_flight = t(2:end, :) ;
    X_flight = X_stance(2:end, :) ;
    u_flight = u_stance(2:end, :) ;
    lambda_flight = lambda_stance(2:end, :) ;
    x0_flight = X_stance(end, :);
    disp('Final stance state (should have velocities):');
    disp(x0_flight)
    
    % Verify we have the full state vector with velocities
    if length(x0_flight) ~= 10
        error('State vector should have 10 elements: [x,y,q1,q2,q3,xdot,ydot,q1dot,q2dot,q3dot]');
    end
    
    % Debug Statement
    fprintf('[Debug] x0 flight velocities: [xdot=%.3f, ydot=%.3f, q1dot=%.3f, q2dot=%.3f, q3dot=%.3f]\n', ...
            x0_flight(6), x0_flight(7), x0_flight(8), x0_flight(9), x0_flight(10));

    % x0(8:10) = 0;  Don't zero out velocities - preserve them from stance phase

    % Time span during flight
    tspan = [t_flight(end), t_flight(end)+3] ;

    % Solve ODE45 during flight
    flight_options = odeset('Events', @(t,x) event_touchdown(t,x,params),'RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01);
    [t_flight, X_flight] = ode45(@(t,x) dynamics_flight(t,x,params), tspan, x0_flight, flight_options);

    % Set the flight input to be 0
    u_flight = zeros(length(t_flight),2);

    % Debug Statement
    disp('[Debug] Flight phase started with initial state:');
    disp(X_flight(1, :));
    fprintf('[Debug] Initial flight y-velocity: %.3f m/s\n', X_flight(1, 7));

    % Plot position, angles, controller during flight
    figure; 
    sgtitle("Flight Phase", 'FontSize', 16, 'FontWeight', 'bold');
    
    subplot(3,2,1); hold on;
    plot(t_flight, X_flight(:,1),'LineWidth',2, "DisplayName", "x")
    plot(t_flight, X_flight(:,2),'LineWidth',2, "DisplayName", "y")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Position (m)");

    subplot(3,2,2); hold on;
    plot(t_flight, X_flight(:,6),'LineWidth',2, "DisplayName", "dx")
    plot(t_flight, X_flight(:,7),'LineWidth',2, "DisplayName", "dy")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Velocity (m/s)");

    subplot(3,2,3); hold on;
    plot(t_flight, X_flight(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_flight, X_flight(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_flight, X_flight(:,5),'LineWidth',2, "DisplayName", "q3")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Angle (rad)");

    subplot(3,2,4); hold on;
    plot(t_flight, X_flight(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_flight, X_flight(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_flight, X_flight(:,10),'LineWidth',2, "DisplayName", "dq3")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Angular Velocity (rad/s)");

    subplot(3,2,5); hold on;
    plot(t_flight, u_flight(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t_flight, u_flight(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Controller Input (N m)");

    % ----------------------- Combine all Stages --------------------------

    % Merge positon, velocity, time, control input 
    X_all = [X_stance; X_flight];
    t_all = [t_stance; t_flight];
    u_all = [u_stance; u_flight];

    % Determine pCOM, pFoot over time
    pCOM = zeros(length(t_all),2);
    pFoot = zeros(length(t_all),2);

    for i = 1:length(t_all)
        pCOM(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1, ...
            params.m1,params.m2,params.m3,X_all(i,3),X_all(i,4), ...
            X_all(i,5),X_all(i,1),X_all(i,2));

        pFoot(i,:) = auto_pfoot(params.l1,params.l2,X_all(i,3),...
            X_all(i,4),X_all(i,1),X_all(i,2));
    end

    % Plot position, angles, controller, pFoot during all stages
    figure; 
    sgtitle("Full Path", 'FontSize', 16, 'FontWeight', 'bold');
    
    subplot(3,2,1); hold on;
    plot(t_all, X_all(:,1),'LineWidth',2, "DisplayName", "x")
    plot(t_all, X_all(:,2),'LineWidth',2, "DisplayName", "y")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Position (m)");

    subplot(3,2,2); hold on;
    plot(t_all, X_all(:,6),'LineWidth',2, "DisplayName", "dx")
    plot(t_all, X_all(:,7),'LineWidth',2, "DisplayName", "dy")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Velocity (m/s)");

    subplot(3,2,3); hold on;
    plot(t_all, X_all(:,3),'LineWidth',2, "DisplayName", "q1")
    plot(t_all, X_all(:,4),'LineWidth',2, "DisplayName", "q2")
    plot(t_all, X_all(:,5),'LineWidth',2, "DisplayName", "q3")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Angle (rad)");

    subplot(3,2,4); hold on;
    plot(t_all, X_all(:,8),'LineWidth',2, "DisplayName", "dq1")
    plot(t_all, X_all(:,9),'LineWidth',2, "DisplayName", "dq2")
    plot(t_all, X_all(:,10),'LineWidth',2, "DisplayName", "dq3")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Angular Velocity (rad/s)");

    subplot(3,2,5); hold on;
    plot(t_all, u_all(:,1),'LineWidth',2, "DisplayName", "u(q1)")
    plot(t_all, u_all(:,2),'LineWidth',2, "DisplayName", "u(q2)")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Controller Input (N m)");

    subplot(3,2,6); hold on;
    plot(t_all, pFoot(:,1),'LineWidth',2, "DisplayName", "Foot_x")
    plot(t_all, pFoot(:,2),'LineWidth',2, "DisplayName", "Foot_y")
    legend('Location', 'best'); xlabel("time (s)"); ylabel("Position (m)");

    % Plot the COM 
    % figure; hold on;
    % plot(t_all, pCOM(:,1),'LineWidth',2, "DisplayName", "COM_x")
    % plot(t_all, pCOM(:,2),'LineWidth',2, "DisplayName", "COM_x")
    % legend('Location', 'best'); xlabel("time (s)"); ylabel("Position (m)");

    % Animate the leg
    animate_leg(t_all, X_all, params);

end