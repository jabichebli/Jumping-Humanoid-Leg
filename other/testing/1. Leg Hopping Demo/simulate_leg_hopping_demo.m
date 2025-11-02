% simulate_leg_hopping_demo.m
function simulate_leg_hopping_demo()
    % -------------------------------------------------------------------------
    % This is a complete, event-driven simulation of a single leg jump.
    % It uses a state machine to switch between stance and flight dynamics
    % based on robust event triggers.
    % -------------------------------------------------------------------------

    % This tells MATLAB where to find the 'auto' functions.
    % addpath('../auto/');
    
    clear all; close all; clc;

    % =========================================================================
    % --- 1. Simulation and Model Parameters ---
    % =========================================================================
    % Model Parameters
    params.m1 = 0.02;   params.m2 = 0.02;   params.m3 = 0.3;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0;
    params.g  = 9.81;
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);

    % Trajectory Parameters (for desired_COMy.m)
    % params.y_stand_target = 0.22; % The desired height for the initial balanced pose
    % params.y_stand = params.y_stand_target;
    % params.y_takeoff      = 0.24;
    % params.T_hold         = 1.0;
    % params.T_jump         = 0.6;  % Total time for the jump motion (dip + push)
    % params.vf_takeoff     = 0.3;  % Desired final velocity

    params.y_stand_target = 0.22; % The desired height for the initial balanced pose
    params.y_stand = params.y_stand_target;
    params.T_jump         = 0.7;  % Total time for the jump motion (dip + push)
    params.y_squat        = 0.06; % The deepest the CoM is allowed to go.
    params.y_takeoff      = 0.26;
    params.T_hold         = 1.0;
    params.vf_takeoff     = 0.5;
    params.T_dip_ratio  = 0.75;  % <-- You can optionally uncomment and tune this!

    params.Kp = 200; params.Kd = 20;

    % Stance Controller Gains
    params.Kp = 200;
    params.Kd = 20;

    % Flight Controller Gains (as 2x1 vectors)
    params.flight_Kp = [2; 2];
    params.flight_Kd = [2; 2];
    params.flight_u_max = [0.2; 0.2];

    % *** FIX 1: Add high-gains for locking joints during flight ***
    params.flight_lock_Kp = [500; 500];
    params.flight_lock_Kd = [50; 50];


    % --- Initial State ---
    % x0 = [0; 0.2970; 0.2; 0.2; 0.0; % Initial positions
    %       0; 0; 0; 0; 0;];                  % Initial velocities
    % x0 = [0; params.y_stand; 1.3; 1.3; 0.0; % Initial positions
    %       0; 0; 0; 0; 0;];                  % Initial velocities

    x0 = calculate_stable_initial_state(params); % Calculate a Stable Initial State
    % visualize_initial_state(x0, params)
    disp(x0)

% =========================================================================
    % --- 2. Event-Driven Simulation Loop ---
    % =========================================================================
    t_start = 0;
    t_end = 8; % Extend simulation time
    x_current = x0;
    
    t_all = []; X_all = []; u_all = []; lambda_all = [];
    t_stance = []; lambda_stance = []; p_d_stance = [];

    opts_stance = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_takeoff1(t,x,params));
    opts_flight = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_touchdown1(t,x,params), 'MaxStep', 1e-2);
    
    % State machine: 1 = Stance (Jumping), 2 = Flight, 3 = Stance (Landing)
    state = 1; 
    
    fprintf('Starting elegant event-driven simulation...\n');
    while t_start < t_end
        switch state
            case 1 % Stance phase, executing the jump trajectory
                fprintf('  State 1: Jumping Stance from t=%.3f\n', t_start);
                [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_stance1(t,x,params), [t_start t_end], x_current, opts_stance);
                
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2); p_d_temp = zeros(length(t),1);
                for i = 1:length(t); [~, u_i, lambda_i, p_d] = dynamics_stance1(t(i), X(i,:)', params); u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; p_d_temp(i) = p_d; end
                t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp]; p_d_stance = [p_d_stance; p_d_temp];
                
                
                next_state = 2; %99; % 2; % Next state is flight
                params.q_des_flight = X(end,3:4);
                
            case 2 % Flight phase
                fprintf('  State 2: Flight from t=%.3f\n', t_start);

                % params.t_start_flight = t_start;

                [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_flight1(t,x,params), [t_start t_end], x_current, opts_flight);
                
                u_temp = zeros(length(t), 2);
                for i = 1:length(t); [~, u_i] = dynamics_flight1(t(i), X(i,:)', params); u_temp(i,:) = u_i'; end

                lambda_temp = zeros(length(t), 2);
                p_d_temp = zeros(length(t),1); % Also zero out the desired COM y-position for flight
                
                next_state = 3; % Next state is landing
                
            case 3 % Stance phase, stabilizing after landing
                fprintf('  State 3: Landing Stance from t=%.3f\n', t_start);
                
                % --- THE FIX ---
                % Instead of simulating to t_end, only simulate for a short
                % period to demonstrate stability.
                t_stabilize_duration = 2.0; % How long to show stabilization (seconds)
                t_integrate_end = min(t_start + t_stabilize_duration, t_end);
                fprintf('    -> Simulating stabilization until t=%.3f...\n', t_integrate_end);
                

                [t, X] = ode45(@(t,x) leg_ode(t,x,params), [t_start t_integrate_end], x_current);
                
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                for i = 1:length(t); [~, u_i, lambda_i] = leg_ode(t(i), X(i,:)', params); u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; end
                t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp]; p_d_stance = [p_d_stance; ones(size(t)) * params.y_stand];
                
                % This is the final phase, so we will exit the loop after this.
                % The 'break' command is handled below.
                next_state = 99; % Use a unique number to signify "done"
        end
        
        % % Store data and transition
        % if isempty(t), t_start = t_end; else t_start = t(end); end
        % t_all = [t_all; t]; X_all = [X_all; X]; u_all = [u_all; u_temp];
        % lambda_all = [lambda_all; lambda_temp];

        % Store data and transition
        if isempty(t)
            t_start = t_end; 
        else
            t_start = t(end);
            
            % --- FIX for interp1 error: Remove duplicate time points ---
            % If t_all is not empty, it means we are appending a new segment.
            % The first point of the new segment (t(1), X(1,:)) is the same 
            % as the last point of the old one, so we remove it.
            if ~isempty(t_all)
                t           = t(2:end);
                X           = X(2:end, :);
                u_temp      = u_temp(2:end, :);
                lambda_temp = lambda_temp(2:end, :);
            end

            % Append the (now unique) data to the master logs
            t_all      = [t_all; t]; 
            X_all      = [X_all; X]; 
            u_all      = [u_all; u_temp];
            lambda_all = [lambda_all; lambda_temp];
        end
        
        % --- THE FIX (Part 2) ---
        % If the state is "done", or if we've reached the end time, break the loop.
        if next_state == 99 || t_start >= t_end
            break; 
        end
        
        x_current = X(end,:)';
        
        % Apply impact dynamics AFTER flight phase but BEFORE the next state begins
        if state == 2 
            fprintf('  Impact at t=%.3f. Recalculating velocities.\n', t_start);
            x_current = dynamics_impact1(x_current, params);
        end
        
        state = next_state;
    end
    fprintf('Simulation finished at t=%.3f\n', t_all(end));

    
    % =========================================================================
    % --- 3. Post-Processing and Plotting ---
    % =========================================================================
    fprintf('Processing and plotting results...\n');
    Fy_all = lambda_all(:, 2);

    pCOM_actual = zeros(length(t_all), 2);
    for i = 1:length(t_all)
       q = X_all(i, 1:5)';
       pCOM_actual(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5),q(1),q(2))';
    end

    % FIGURE 1: Subplots of Key Dynamics
    figure('Name', 'Jumping Dynamics Analysis', 'Position', [100, 100, 800, 900]);
    subplot(3,1,1);
    plot(t_all(1:end), X_all(1:end,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Hip X'); hold on;
    plot(t_all(1:end), X_all(1:end,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Hip Y');
    grid on; title('Hip Position vs. Time'); xlabel('Time (s)'); ylabel('Position (m)'); legend;
    subplot(3,1,2);
    plot(t_all(1:end), Fy_all(1:end), 'k-', 'LineWidth', 2);
    grid on; title('Vertical Ground Reaction Force vs. Time'); xlabel('Time (s)'); ylabel('Force (N)');
    subplot(3,1,3);
    plot(t_all(1:end), u_all(1:end,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Torque 1'); hold on;
    plot(t_all(1:end), u_all(1:end,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Torque 2');
    grid on; title('Motor Torques vs. Time'); xlabel('Time (s)'); ylabel('Torque (Nm)'); legend;

    % FIGURE 2: COM Height Tracking
    figure('Name', 'COM Height Tracking');
    plot(t_stance, p_d_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM Height'); hold on;
    plot(t_all, pCOM_actual(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM Height');
    grid on; title('Center of Mass Height vs. Time'); xlabel('Time (s)'); ylabel('Height (m)'); legend;

    % --- FIGURE 3: Joint Space Debugging ---
    figure('Name', 'Joint Space Analysis');
    subplot(2,1,1);
    plot(t_all(1:end), X_all(1:end,3), 'r-', 'LineWidth', 2, 'DisplayName', 'q1 (Hip)'); hold on;
    plot(t_all(1:end), X_all(1:end,4), 'b-', 'LineWidth', 2, 'DisplayName', 'q2 (Knee)');
    grid on; title('Joint Angles'); ylabel('Angle (rad)'); legend;
    subplot(2,1,2);
    plot(t_all(1:end), X_all(1:end,8), 'r-', 'LineWidth', 2, 'DisplayName', 'q1dot'); hold on;
    plot(t_all(1:end), X_all(1:end,9), 'b-', 'LineWidth', 2, 'DisplayName', 'q2dot');
    grid on; title('Joint Velocities'); xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)'); legend;

    % --- ANIMATION ---
    fprintf('Generating animation... (this may take a moment)\n');
    
    % ---------------- Plot & Animate ----------------
    figure; hold on;
    plot(t_all, X_all(:,2), 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Vertical COM position (m)');
    title('Repeated Hopping (10 cycles)');
    grid on;

    animate_hopping_gif(t_all, X_all, params); % **run me to visualize** 

    % animate_leg(t_all, X_all, params); % Dont run, run the above
    % animate_hopping_gif if u want the flight path to be same time as
    % rest, code has not been updated to make it consistent

    
end

% =========================================================================
% --- ROBUST HELPER FUNCTIONS for STABLE INITIALIZATION ---
% =========================================================================
function x0 = calculate_stable_initial_state(params)
    fprintf('Calculating a stable initial state...\n');

    ik_errors = @(vars) ik_stability_helper(vars, params);
    initial_guess = [1.3; 1.3; 0.0; params.y_stand_target]; 
    options = optimoptions('fsolve', 'Display', 'none');
    solution = fsolve(ik_errors, initial_guess, options);
    
    q1_sol = solution(1);
    q2_sol = solution(2);
    hip_x_sol = solution(3);
    hip_y_sol = solution(4);
    
    x0 = [hip_x_sol; hip_y_sol; q1_sol; q2_sol; 0.0; % Positions
          0; 0; 0; 0; 0;];                         % Velocities
          
    fprintf('  -> Stable state found: y_hip=%.4f, x_hip=%.4f, q1=%.3f, q2=%.3f\n', x0(2), x0(1), x0(3), x0(4));
end

function errors = ik_stability_helper(vars, params)
    q1    = vars(1);
    q2    = vars(2);
    hip_x = vars(3);
    hip_y = vars(4);
    
    pFoot = auto_pfoot(params.l1, params.l2, q1, q2, hip_x, hip_y);
    pCOM  = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,0,hip_x,hip_y);

    error_foot_y = pFoot(2);
    error_foot_x = pFoot(1);
    error_com_x  = pCOM(1);
    error_com_y  = pCOM(2) - params.y_stand_target;
    
    errors = [error_foot_y; error_foot_x; error_com_x; error_com_y];
end

% % =========================================================================
% % --- HELPER FUNCTIONS ---
% % =========================================================================
% function [x0, params] = calculate_initial_state(params)
%     fprintf('Calculating a stable initial state...\n');
% 
%     ik_errors = @(q_vars) ik_helper(q_vars, params);
%     q_initial_guess = [1.3; 1.3];
%     options = optimoptions('fsolve', 'Display', 'none', 'FunctionTolerance', 1e-12);
%     q_sol = fsolve(ik_errors, q_initial_guess, options);
% 
%     % --- DEFINITIVE FIX: Adjust the hip position to enforce a perfect stance ---
%     % 1. Calculate the small residual foot position error from the solver.
%     hip_y_guess = params.y_stand_target;
%     foot_pos_error = auto_pfoot(params.l1, params.l2, q_sol(1), q_sol(2), 0, hip_y_guess);
% 
%     % 2. Adjust the hip position to perfectly cancel out this error.
%     final_hip_x = 0 - foot_pos_error(1);
%     final_hip_y = hip_y_guess - foot_pos_error(2);
% 
%     % 3. CRITICAL: Update the y_stand parameter for the entire simulation to this
%     %    new, physically consistent height.
%     params.y_stand = final_hip_y;
% 
%     % 4. Construct the final, perfect initial state vector.
%     x0 = [final_hip_x; final_hip_y; q_sol(1); q_sol(2); 0.0; % Positions
%           0; 0; 0; 0; 0;];                                 % Velocities
% 
%     fprintf('  -> Solver residual error was [%.2e, %.2e]\n', foot_pos_error(1), foot_pos_error(2));
%     fprintf('  -> Adjusted params.y_stand to %.4f m to compensate.\n', params.y_stand);
%     fprintf('  -> Final Initial State: y=%.4f, q1=%.3f, q2=%.3f\n', x0(2), x0(3), x0(4));
% end
% 
% function errors = ik_helper(q_vars, params)
%     q1 = q_vars(1);
%     q2 = q_vars(2);
%     foot_pos_relative = [-params.l1*sin(q1) + params.l2*sin(q2 - q1);
%                          -params.l1*cos(q1) - params.l2*cos(q2 - q1)];
%     target_pos_relative = [0; -params.y_stand_target];
%     errors = foot_pos_relative - target_pos_relative;
% end

function visualize_initial_state(x0, params)
    % (This function is unchanged but will now show the correct pose)
    figure('Name', 'Initial State Visualization');
    q = x0(1:5);
    x=q(1); y=q(2); q1=q(3); q2=q(4); q3=q(5);
    hip = [x; y]; knee = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
    foot = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
    torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)];
    pCOM = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,q3,x,y);
    hold on; grid on; axis equal;
    plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth',2.5, 'MarkerSize', 8);
    plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth',2.5, 'MarkerSize', 8);
    com_marker_radius = 0.02; com_marker_color = 'k'; com_line_width = 1.5;
    com_x = pCOM(1); com_y = pCOM(2);
    theta = 0:0.1:2*pi;
    plot(com_x + com_marker_radius*cos(theta), com_y + com_marker_radius*sin(theta), 'Color', com_marker_color, 'LineWidth', com_line_width);
    plot([com_x - com_marker_radius, com_x + com_marker_radius], [com_y, com_y], 'Color', com_marker_color, 'LineWidth', com_line_width);
    plot([com_x, com_x], [com_y - com_marker_radius, com_y + com_marker_radius], 'Color', com_marker_color, 'LineWidth', com_line_width);
    line([-0.5, 0.5], [0, 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
    title('Calculated Initial State (Press any key to continue)');
    xlabel('X (m)'); ylabel('Y (m)');
    axis([-0.4 0.4 -0.1 0.5]);
    drawnow;
    pause;
end