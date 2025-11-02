function J = cost_function_flip(p)
    % This function calculates the "cost" of a flip for a given set of
    % decision variables 'p'.

    % 1. Define base parameters (can be loaded or defined here)
    params.m1 = 0.02;   params.m2 = 0.02;   params.m3 = 0.3;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.05;
    params.g  = 9.81;
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);
    % ... (all other non-optimized params) ...
    params.Kp = 50; params.Kd = 5;
    params.flight_Kp = [50; 50]; params.flight_Kd = [5; 5]; params.flight_u_max = [2.0; 2.0];
    params.q_tuck_des = [deg2rad(270); deg2rad(170)];
    params.y_stand_target = 0.22; params.y_stand = params.y_stand_target;
    params.T_jump = 1.0; params.y_squat = 0.10; params.y_takeoff = 0.24;
    params.T_hold = 1.0; params.T_dip_ratio = 0.5;

    % 2. Unpack decision variables 'p' into the params struct
    params.q3_takeoff         = p(1);
    params.q3dot_takeoff      = p(2);
    params.vf_takeoff         = p(3);
    params.flip_timing_ratio  = p(4);
    
    % 3. Run the simulation
    % % try
        % The simulation can fail if the parameters are bad (e.g., robot falls)
        % The 'try/catch' block prevents the optimizer from crashing.
        [~, X_all, u_all, pCOM_actual] = run_flip_simulation(params);
        
        % Check if the simulation even completed a flip (landed in state 3)
        % if size(X_all, 1) < 10 % Arbitrary short length, means it crashed early
        %     J = 1e9; % Assign a huge cost to failed simulations
        %     fprintf('Simulation failed early. Cost = %.1e\n', J);
        %     return;
        % end
        
    % catch
    %     % If any error occurs during the simulation
    %     J = 1e9; % Assign a huge cost
    %     fprintf('Simulation threw an error. Cost = %.1e\n', J);
    %     return;
    % end
    
    % 4. Calculate the components of the cost function
    final_state = X_all(end, :);
    final_pCOM = pCOM_actual(end, :);
    
    % Define the target landing state for a FRONT FLIP
    target_q3 = -2 * pi;
    
    % --- Cost Terms ---
    angle_error    = final_state(5) - target_q3;
    velocity_error = final_state(10); % Target is 0
    drift_error    = final_pCOM(1);   % Target is 0
    effort         = sum(sum(u_all.^2));
    
    % --- Weights (TUNE THESE!) ---
    w_angle = 100.0;  % Primary goal
    w_vel   = 10.0;
    w_drift = 5.0;
    w_effort = 0.01;
    
    % 5. Calculate the final scalar cost
    J = w_angle * angle_error^2 ...
      + w_vel * velocity_error^2 ...
      + w_drift * drift_error^2 ...
      + w_effort * effort;
      
    fprintf('Vars: q3_f=%.2f, q3d_f=%.2f, vf=%.2f, ratio=%.2f  -->  Cost: %.4f\n', ...
            p(1), p(2), p(3), p(4), J);
end