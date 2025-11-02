% simulate_leg_leaping_demo.m
function simulate_leg_leaping_demo()
    % -------------------------------------------------------------------------
    % This is a complete, event-driven simulation of a single leg FRONT FLIP.
    % -------------------------------------------------------------------------
    
    % addpath('../auto/'); % Ensure auto functions are on path
    clear all; close all; clc;

    % =========================================================================
    % --- 1. Simulation and Model Parameters ---
    % =========================================================================
    % Model Parameters (same as before)
    params.m1 = 0.02;   params.m2 = 0.02;   params.m3 = 0.3;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0;
    params.g  = 9.81;
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);

    % --- Trajectory Parameters ---
    params.y_stand_target = 0.22;
    params.y_stand        = params.y_stand_target;
    params.T_jump         = 0.7;  % Total time for the jump motion
    params.y_squat        = 0.06; % The deepest the CoM is allowed to go.
    params.y_takeoff      = 0.26;
    params.T_hold         = 0.6; %0.6;
    params.vf_takeoff     = 0.6; % 0.6;
    params.T_dip_ratio    = 0.75;  % 75% of jump time spent on dip/lean




    % --- FLIP PARAMETER ---
    params.x_squat_lean = 0.0;% 0.04; % The lean at the BOTTOM of the squat (-0.01)
    params.x_takeoff = 0.0; % 0.18; % The lean at takeoff (-0.06)

    
    % Controller Gains
    params.Kp = 200; params.Kd = 20;

%    params.Kp = 200;
%    params.Kd = 40;
   

    params.flight_Kp = [2; 2];
    params.flight_Kd = [2; 2];
    params.flight_u_max = [0.2; 0.2];

    % --- Initial State ---
    x0 = calculate_stable_initial_state(params);
    disp('Initial State:');
    disp(x0);

    % =========================================================================
    % --- 2. Event-Driven Simulation Loop ---
    % =========================================================================
    t_start = 0;
    t_end = 8;
    x_current = x0;
    
    t_all = []; X_all = []; u_all = []; lambda_all = [];
    t_stance = []; lambda_stance = []; p_d_y_stance = []; p_d_x_stance = [];

    % Use the NEW event functions
    opts_stance = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_takeoff2(t,x,params));
    opts_flight = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_touchdown2(t,x,params), 'MaxStep', 1e-2);
    
    state = 1; % 1 = Stance (Leaping), 2 = Flight, 3 = Stance (Landing)
    
    fprintf('Starting front flip simulation...\n');
    while t_start < t_end
        switch state
            case 1 % Stance phase, executing the flip
                fprintf('  State 1: Leaping Stance from t=%.3f\n', t_start);
                [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_stance2(t,x,params), [t_start t_end], x_current, opts_stance);
                
                % Log data from stance controller
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                p_d_y_temp = zeros(length(t),1); p_d_x_temp = zeros(length(t),1);
                for i = 1:length(t)
                    [~, u_i, lambda_i, p_d] = dynamics_stance2(t(i), X(i,:)', params);
                    u_temp(i,:) = u_i'; 
                    lambda_temp(i,:) = lambda_i'; 
                    p_d_x_temp(i) = p_d(1);
                    p_d_y_temp(i) = p_d(2);
                end
                t_stance = [t_stance; t]; 
                lambda_stance = [lambda_stance; lambda_temp]; 
                p_d_y_stance = [p_d_y_stance; p_d_y_temp];
                p_d_x_stance = [p_d_x_stance; p_d_x_temp];
                
                next_state = 2;
                params.q_des_flight = X(end,3:4); % Set flight leg angles
                
            case 2 % Flight phase
                fprintf('  State 2: Flight from t=%.3f\n', t_start);
                [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_flight2(t,x,params), [t_start t_end], x_current, opts_flight);
                
                u_temp = zeros(length(t), 2);
                for i = 1:length(t); [~, u_i] = dynamics_flight2(t(i), X(i,:)', params); u_temp(i,:) = u_i'; end
                lambda_temp = zeros(length(t), 2);
                
                next_state = 3;
                
            case 3 % Stance phase, stabilizing after landing
                fprintf('  State 3: Landing Stance from t=%.3f\n', t_start);
                t_stabilize_duration = 2.0;
                t_integrate_end = min(t_start + t_stabilize_duration, t_end);

                [t, X] = ode45(@(t,x) dynamics_stabilize2(t,x,params), [t_start t_integrate_end], x_current);
                
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                for i = 1:length(t); [~, u_i, lambda_i] = dynamics_stabilize2(t(i), X(i,:)', params); u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; end
                
                % Log stabilization data
                t_stance = [t_stance; t]; 
                lambda_stance = [lambda_stance; lambda_temp]; 
                p_d_y_stance = [p_d_y_stance; ones(size(t)) * params.y_stand]; % Stabilize goal
                p_d_x_stance = [p_d_x_stance; zeros(size(t))]; % Stabilize goal
                
                next_state = 99; % Done
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
        
        if next_state == 99 || t_start >= t_end
            break; 
        end
        
        x_current = X(end,:)';
        
        % Apply impact dynamics AFTER flight
        if state == 2 
            fprintf('  Impact at t=%.3f. Recalculating velocities.\n', t_start);
            x_current = dynamics_impact2(x_current, params);
        end
        
        state = next_state;
    end
    fprintf('Simulation finished at t=%.3f\n', t_all(end));

    
    % =========================================================================
    % --- 3. Post-Processing and Plotting ---
    % =========================================================================
    fprintf('Processing and plotting results...\n');
    
    pCOM_actual = zeros(length(t_all), 2);
    for i = 1:length(t_all)
       q = X_all(i, 1:5)';
       pCOM_actual(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5),q(1),q(2))';
    end

    % FIGURE 1: COM Height and Torso Angle Tracking
    % figure('Name', 'Flip Controller Tracking');
    % subplot(2,1,1);
    % plot(t_stance, p_d_y_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM-Y'); hold on;
    % plot(t_all, pCOM_actual(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-Y');
    % grid on; title('Center of Mass Height vs. Time'); xlabel('Time (s)'); ylabel('Height (m)'); legend;
    % 
    % subplot(2,1,2);
    % plot(t_stance, p_d_q3_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired Torso q3'); hold on;
    % plot(t_all, X_all(:,5), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Torso q3');
    % grid on; title('Torso Angle vs. Time'); xlabel('Time (s)'); ylabel('Angle (rad)'); legend;
    
    % FIGURE 1: COM Tracking
    figure('Name', 'Flip Controller Tracking');
    subplot(2,1,1);
    plot(t_stance, p_d_y_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM-Y'); hold on;
    plot(t_all, pCOM_actual(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-Y');
    grid on; title('Center of Mass Height vs. Time'); ylabel('Height (m)'); legend;
    
    subplot(2,1,2);
    plot(t_stance, p_d_x_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM-X'); hold on;
    plot(t_all, pCOM_actual(:,1), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-X');
    grid on; title('Center of Mass Lean vs. Time'); xlabel('Time (s)'); ylabel('Position (m)'); legend;

    % FIGURE 2: Torso Angle (to see the flip)
    figure('Name', 'Flip Result');
    plot(t_all, X_all(:,5), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Torso q3');
    grid on; title('Torso Angle vs. Time'); xlabel('Time (s)'); ylabel('Angle (rad)'); legend;

    % FIGURE 2: Forces and Torques
    figure('Name', 'Flip Dynamics Analysis', 'Position', [100, 100, 800, 900]);
    subplot(3,1,1);
    plot(t_all, X_all(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Hip X'); hold on;
    plot(t_all, pCOM_actual(:,1), 'k-', 'LineWidth', 2, 'DisplayName', 'COM X');
    grid on; title('Horizontal Position vs. Time'); xlabel('Time (s)'); ylabel('Position (m)'); legend;
    subplot(3,1,2);
    plot(t_all, lambda_all(:, 2), 'k-', 'LineWidth', 2);
    grid on; title('Vertical Ground Reaction Force vs. Time'); xlabel('Time (s)'); ylabel('Force (N)');
    subplot(3,1,3);
    plot(t_all, u_all(:,1), 'r-', 'LineWidth', 2, 'DisplayName', 'Torque 1'); hold on;
    plot(t_all, u_all(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Torque 2');
    grid on; title('Motor Torques vs. Time'); xlabel('Time (s)'); ylabel('Torque (Nm)'); legend;

    % --- ANIMATION ---
    fprintf('Generating animation... (this may take a moment)\n');
    % You can reuse your animation function
    animate_leaping_gif(t_all, X_all, params, 'leg_leaping_animation.gif');
    
end

% =========================================================================
% --- ROBUST HELPER FUNCTIONS for STABLE INITIALIZATION ---
% (These are copied directly from simulate_leg_jumping_demo.m)
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
    
    x0 = [hip_x_sol; hip_y_sol; q1_sol; q2_sol; 0.0; % Positions (q3=0)
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