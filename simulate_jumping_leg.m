% -------------------------------------------------------------------------
% simulate_jumping_leg.m
% -------------------------------------------------------------------------
% This is a complete, event-driven simulation of a single leg jumping. By
% changing the takeoff parameters, you can change whether it jumps 
% forwards, backwards or straight up. 
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function simulate_jumping_leg()
    % ---------------------------------------------------------------------
    % If the auto folder is not generated, you need to generate it first.
    % To do so, you need to run: generate_files.m
    % This should create and populate an auto folder. This only ever needs
    % to be done once (Unless you change the dynamics in the
    % generate_files.m, then you will need to regenerate it).
    % ---------------------------------------------------------------------

    clear all; close all; clc;

    % =====================================================================
    % ------------ 0.  Add the path for the different folders -------------
    % Note: you don't need to run this everytime, only at the start 
    % (unless you change folders). Uncomment the code first time you run.
    % =====================================================================
    
    addpath('./auto/');
    addpath('./trajectory/');
    addpath('./events/'); 
    addpath('./dynamics/');
    addpath('./animate/'); 

    % =====================================================================
    % ------------- 1. Define the simulation parameters -------------------
    % Note: This is where you change the code to make the leg jump:
    % upwards, forwards, or backwards.
    % =====================================================================
    
    % Model Parameters
    params.m1 = 0.02;   params.m2 = 0.02;   params.m3 = 0.3;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;  
    params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0; % d3 = 0 is important! 
    params.g  = 9.81;
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);
    params.mu = 0.8;

    % Trajectory parameters - main
    params.y_stand_target = 0.22;
    params.y_stand        = params.y_stand_target;
    params.T_jump         = 0.7;  % Total time for the jump motion
    params.y_squat        = 0.06; % The deepest the CoM is allowed to go.
    params.y_takeoff      = 0.26;
    params.T_hold         = 0.6; 
    params.vf_takeoff     = 0.6; % vertical takeoff speed
    params.T_dip_ratio    = 0.75; % 75% of jump time spent on dip/lean

    % Trajectory parameters - forward/backwards/upwards only 
    % x_squat_lean: upwards: 0.0;   forward: 0.04;   backwards: -0.01
    % x_takeoff:    upwards: 0.0;   forward: 0.18;   backwards: -0.06
    params.x_squat_lean = 0.0;  % The lean at the bottom of the squat  
    params.x_takeoff = 0.0;     % The lean at takeoff

    
    % Controller Gains
    params.Kp = 200; params.Kd = 20; % Stance controller
    params.flight_Kp = [2; 2]; params.flight_Kd = [2; 2]; % Flight controller
    params.flight_u_max = [0.2; 0.2]; % Flight Controller limits

    % Initial State (Stable)
    x0 = calculate_stable_initial_state(params);
    % disp('Initial State:');
    % disp(x0);

    % =====================================================================
    % --------------- 2. Event-Driven Simulation Loop ---------------------
    % Note: This simulates the one-cycle fo jumping.
    % =====================================================================
    
    % Initialize simulation parameters
    t_start = 0; t_end = 8; x_current = x0;
    t_all = []; X_all = []; u_all = []; lambda_all = [];
    t_stance = []; lambda_stance = []; p_d_y_stance = []; p_d_x_stance = [];

    % Define the event functions 
    opts_stance = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_takeoff(t,x,params));
    opts_flight = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_touchdown(t,x,params), 'MaxStep', 1e-2);
    
    state = 1; % 1 = Stance (Liftoff), 2 = Flight, 3 = Stance (Landing)
    
    fprintf('Starting jumping simulation...\n');
    while t_start < t_end
        switch state
            case 1 % Stance Phase: Stablizing, Squating and Takeoff
                fprintf('  State 1: Liftoff Stance from t=%.3f\n', t_start);
                [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_stance(t,x,params), [t_start t_end], x_current, opts_stance);
                Ok,
                % Log liftoff data
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                p_d_y_temp = zeros(length(t),1); p_d_x_temp = zeros(length(t),1);
                for i = 1:length(t)
                    [~, u_i, lambda_i, p_d] = dynamics_stance(t(i), X(i,:)', params);
                    u_temp(i,:) = u_i'; 
                    lambda_temp(i,:) = lambda_i'; 
                    p_d_x_temp(i) = p_d(1);
                    p_d_y_temp(i) = p_d(2);
                end
                t_stance = [t_stance; t]; 
                lambda_stance = [lambda_stance; lambda_temp]; 
                p_d_y_stance = [p_d_y_stance; p_d_y_temp];
                p_d_x_stance = [p_d_x_stance; p_d_x_temp];
                
                % Move onto next phase
                next_state = 2;
                params.q_des_flight = X(end,3:4); % Set flight leg angles
                
            case 2 % Flight Phase
                fprintf('  State 2: Flight from t=%.3f\n', t_start);
                [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_flight(t,x,params), [t_start t_end], x_current, opts_flight);
                
                % Log flight data
                u_temp = zeros(length(t), 2);
                for i = 1:length(t);
                    [~, u_i] = dynamics_flight(t(i), X(i,:)', params);
                    u_temp(i,:) = u_i';
                end
                lambda_temp = zeros(length(t), 2);
                
                % Move onto next phase
                next_state = 3;
                
            case 3 % Stance Phase: stabilizing after landing
                fprintf('  State 3: Landing Stance from t=%.3f\n', t_start);
                t_stabilize_duration = 2.0; t_integrate_end = min(t_start + t_stabilize_duration, t_end);
                [t, X] = ode45(@(t,x) dynamics_stabilize(t,x,params), [t_start t_integrate_end], x_current);
                
                % Log stabilization data
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                for i = 1:length(t)
                    [~, u_i, lambda_i] = dynamics_stabilize(t(i), X(i,:)', params);
                    u_temp(i,:) = u_i'; 
                    lambda_temp(i,:) = lambda_i';
                end

                t_stance = [t_stance; t]; 
                lambda_stance = [lambda_stance; lambda_temp]; 
                p_d_y_stance = [p_d_y_stance; ones(size(t)) * params.y_stand];
                p_d_x_stance = [p_d_x_stance; zeros(size(t))];
                
                % Move onto next phase (Done)
                next_state = 99; 
        end
        
        % Store data and transition
        if isempty(t)
            t_start = t_end; 
        else
            if ~isempty(t_all) % Remove duplicate time points
                t = t(2:end);
                X = X(2:end, :);
                u_temp = u_temp(2:end, :);
                lambda_temp = lambda_temp(2:end, :);
            end
            t_all = [t_all; t]; 
            X_all = [X_all; X]; 
            u_all = [u_all; u_temp];
            lambda_all = [lambda_all; lambda_temp];
            t_start = t(end);
        end
        
        % Check if we reached the end
        if next_state == 99 || t_start >= t_end
            break; 
        end

        x_current = X(end,:)';
        
        % Apply impact dynamics after flight
        if state == 2 
            fprintf('  Impact at t=%.3f. Recalculating velocities.\n', t_start);
            x_current = dynamics_impact(x_current, params);
        end
        
        state = next_state;
    end
    fprintf('Simulation finished at t=%.3f\n', t_all(end));

    
    % =====================================================================
    % --------------- 3. Post-Processing and Plotting ---------------------
    % Note: This is where the different plots and animations are called.
    % =====================================================================
    fprintf('Processing and plotting results...\n');
    
    % Determine the COM of the system throughout the jump
    pCOM_actual = zeros(length(t_all), 2);
    for i = 1:length(t_all)
       q = X_all(i, 1:5)';
       pCOM_actual(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5),q(1),q(2))';
    end

    % FIGURE 1: Position (x,y), Velocity, Angles, Angular Velocity during all stages
    figure; 
    sgtitle("Figure 1: System States over Time", 'FontSize', 16, 'FontWeight', 'bold');
    
    subplot(2,2,1); hold on;
    plot(t_all, X_all(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'x');
    plot(t_all, X_all(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'y');
    grid on;
    title('Position vs. Time');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('Location', 'best');
    hold off;
    
    subplot(2,2,2); hold on;
    plot(t_all, X_all(:,6), 'r-', 'LineWidth', 2, 'DisplayName', 'dx');
    plot(t_all, X_all(:,7), 'b-', 'LineWidth', 2, 'DisplayName', 'dy');
    grid on;
    title('Linear Velocity vs. Time');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('Location', 'best');
    hold off;
    
    subplot(2,2,3); hold on;
    plot(t_all, rad2deg(X_all(:,3)), 'r-', 'LineWidth', 2, 'DisplayName', 'q_1');
    plot(t_all, rad2deg(X_all(:,4)), 'b-', 'LineWidth', 2, 'DisplayName', 'q_2');
    plot(t_all, rad2deg(X_all(:,5)), 'g-', 'LineWidth', 2, 'DisplayName', 'q_3');
    grid on;
    title('Joint Angles vs. Time');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('Location', 'best');
    hold off;
    
    subplot(2,2,4); hold on;
    plot(t_all, rad2deg(X_all(:,8)), 'r-', 'LineWidth', 2, 'DisplayName', 'dq_1');
    plot(t_all, rad2deg(X_all(:,9)), 'b-', 'LineWidth', 2, 'DisplayName', 'dq_2');
    plot(t_all, rad2deg(X_all(:,10)), 'g-', 'LineWidth', 2, 'DisplayName', 'dq_3');
    grid on;
    title('Angular Velocities vs. Time');
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    legend('Location', 'best');
    hold off;

    % FIGURE 2: Forces and Torques
    figure;
    sgtitle("Figure 2: Torque and GRFs", 'FontSize', 16, 'FontWeight', 'bold');
    
    % No Lift-off Condition
    subplot(3,1,1);
    Fy = lambda_all(:,2);
    plot(t_all, Fy, 'b-', 'LineWidth', 2, 'DisplayName', 'F_y (must be > 0)');
    hold on;
    yline(0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Lift-off Limit');
    grid on;
    title('Vertical Ground Reaction Force (No Lift-off)');
    xlabel('Time (s)');
    ylabel('F_y (N)');
    legend('Location', 'best');
    hold off;
    
    % No Slip Condition
    subplot(3,1,2);
    Fx = lambda_all(:,1);
    mu = params.mu;
    ratio = abs(Fx) ./ Fy;
    plot(t_all, ratio, 'b-', 'LineWidth', 2, 'DisplayName', 'Ratio (must be < \mu)');
    hold on;
    yline(mu, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Friction Limit \mu');
    grid on;
    title('Friction Constraint (No Slip)');
    xlabel('Time (s)');
    ylabel('$|F_x| / F_y$', 'Interpreter', 'latex');
    legend('Location', 'best');
    ylim([0, mu*2]);
    hold off;
    
    subplot(3,1,3);
    plot(t_all, u_all(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Torque 1'); hold on;
    plot(t_all, u_all(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Torque 2');
    grid on;
    title('Motor Torques vs. Time');
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    legend('Location', 'best');
        
    % FIGURE 3: COM Tracking
    figure;
    sgtitle("Figure 3: Center of Mass", 'FontSize', 16, 'FontWeight', 'bold');

    subplot(2,1,1);
    plot(t_stance, p_d_y_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM-Y'); hold on;
    plot(t_all, pCOM_actual(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-Y');
    grid on; title('Center of Mass Height vs. Time'); ylabel('Height (m)'); legend;
    
    subplot(2,1,2);
    plot(t_stance, p_d_x_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM-X'); hold on;
    plot(t_all, pCOM_actual(:,1), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-X');
    grid on; title('Center of Mass Lean vs. Time'); xlabel('Time (s)'); ylabel('Position (m)'); legend;


    % ANIMATION
    fprintf('Generating animation... (this may take a moment)\n');
    animate_jumping(t_all, X_all, params, "realtime"); % To visualize the animation
    % animate_jumping(t_all, X_all, params, "gif", "media/leg_jumping_animation.gif");  % To save the animation as gif

end


% =========================================================================
% ---------- Robust Helper Functions for stable initialization ------------
% Note: This is where the different plots and animations are called.
% =========================================================================

function x0 = calculate_stable_initial_state(params)
    % ---------------------------------------------------------------------
    % Computes a stable initial configuration (x0) for the simulation,
    % ensuring the leg starts balanced and with zero joint velocities.
    %
    % Inputs:
    %   params - system parameters
    %
    % Outputs:
    %   x0     - initial state vector [q; dq], where:
    %               q = [hip_x; hip_y; q1; q2; q3]
    %               dq = [dx; dy; dq1; dq2; dq3]
    % ---------------------------------------------------------------------

    fprintf('Calculating a stable initial state...\n');

    % Define the system of nonlinear equations to solve (inverse kinematics)
    ik_errors = @(vars) ik_stability_helper(vars, params);

    % Initial guess: [q1; q2; hip_x; hip_y]
    initial_guess = [1.3; 1.3; 0.0; params.y_stand_target]; 

    % Use fsolve for numerical root-finding
    options = optimoptions('fsolve', 'Display', 'none');
    solution = fsolve(ik_errors, initial_guess, options);
    
    % Extract solved joint angles and hip coordinates
    q1_sol = solution(1);
    q2_sol = solution(2);
    hip_x_sol = solution(3);
    hip_y_sol = solution(4);
    
    % Construct full state vector: positions + zero velocities
    x0 = [hip_x_sol; hip_y_sol; q1_sol; q2_sol; 0.0; % Positions (q3=0)
          0; 0; 0; 0; 0;];                           % Velocities
          
    fprintf('  -> Stable state found: y_hip=%.4f, x_hip=%.4f, q1=%.3f, q2=%.3f\n', x0(2), x0(1), x0(3), x0(4));
end

function errors = ik_stability_helper(vars, params)
    % ---------------------------------------------------------------------
    % Helper function for fsolve that computes the residual errors in foot 
    % and COM positions given a candidate joint configuration.
    %
    % Inputs:
    %   vars   - vector of unknowns [q1; q2; hip_x; hip_y]
    %   params - system parameters used by auto_pfoot() and auto_pCOM()
    %
    % Outputs:
    %   errors - vector of position errors used by fsolve:
    %               [foot_y; foot_x; COM_x; COM_y_error]
    % ---------------------------------------------------------------------

    q1    = vars(1);
    q2    = vars(2);
    hip_x = vars(3);
    hip_y = vars(4);
    
    % Compute foot position given current joint configuration
    pFoot = auto_pfoot(params.l1, params.l2, q1, q2, hip_x, hip_y);

    % Compute COM position based on link geometry and masses
    pCOM  = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,0,hip_x,hip_y);

    % Define positional error terms for fsolve
    error_foot_y = pFoot(2);                        % Ensure foot touches ground
    error_foot_x = pFoot(1);                        % Align foot horizontally
    error_com_x  = pCOM(1);                         % Keep COM centered
    error_com_y  = pCOM(2) - params.y_stand_target; % Match target COM height
    
    errors = [error_foot_y; error_foot_x; error_com_x; error_com_y];
end