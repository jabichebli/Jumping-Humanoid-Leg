function [t_all, X_all, u_all, pCOM_actual] = run_flip_simulation(params)
    % -------------------------------------------------------------------------
    % This function runs a single simulation for the flip controller
    % given a set of parameters. It is designed to be called by an
    % optimizer and returns the simulation results for cost calculation.
    % It contains NO plotting or animation.
    %
    % It uses the STIFF solver 'ode15s' for stability.
    % -------------------------------------------------------------------------
    
    % --- Initial State ---
    % We assume 'auto_...' functions are on the path
    try
        x0 = calculate_stable_initial_state(params);
    catch ME
        % If IK solver fails, this is a bad set of params
        fprintf('  calculate_stable_initial_state failed: %s\n', ME.message);
        t_all = []; X_all = []; u_all = []; pCOM_actual = [];
        return;
    end
    
    % =========================================================================
    % --- 2. Event-Driven Simulation Loop ---
    % =========================================================================
    t_start = 0;
    t_end = 8;
    x_current = x0;
    
    % Initialize data storage (CRITICAL to prevent 't_all' error)
    t_all = []; X_all = []; u_all = []; lambda_all = [];
    t_stance = []; lambda_stance = []; p_d_y_stance = []; p_d_q3_stance = [];
    
    % Setup ODE options
    opts_stance = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_takeoff(t,x,params));
    opts_flight = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_touchdown(t,x,params), 'MaxStep', 1e-2);
    
    state = 1;
    
    % fprintf('Starting STABLE flip simulation (tracking q3)...\n');
    while t_start < t_end
        switch state
            case 1 % Stance phase, executing the flip
                % fprintf('  State 1: Flipping Stance from t=%.3f\n', t_start);
                
                % --- CRITICAL FIX: Use ode15s (stiff solver) instead of ode45 ---
                [t, X, ~, ~, ~] = ode15s(@(t,x) dynamics_stance(t,x,params), [t_start t_end], x_current, opts_stance);
                
                % Check for immediate failure from ode15s
                if isempty(t)
                    % fprintf('ODE solver failed in state 1.\n');
                    break; % Exit switch and let loop handle empty 't'
                end
                
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                p_d_y_temp = zeros(length(t),1); p_d_q3_temp = zeros(length(t),1);
                
                for i = 1:length(t)
                    % Recalculate dynamics to get u and lambda
                    [~, u_i, lambda_i, p_d_y, p_d_q3] = dynamics_stance(t(i), X(i,:)', params);
                    u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i';
                    p_d_y_temp(i) = p_d_y; p_d_q3_temp(i) = p_d_q3;
                end
                t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp];
                p_d_y_stance = [p_d_y_stance; p_d_y_temp]; p_d_q3_stance = [p_d_q3_stance; p_d_q3_temp];
                
                next_state = 2;
                if ~isempty(X)
                    params.q_des_flight = X(end,3:4);
                end
                
            case 2 % Flight phase
                % fprintf('  State 2: Flight from t=%.3f\n', t_start);
                
                % --- Use ode15s here too for consistency ---
                [t, X, ~, ~, ~] = ode15s(@(t,x) dynamics_flight(t,x,params), [t_start t_end], x_current, opts_flight);
                
                if isempty(t)
                    % fprintf('ODE solver failed in state 2.\n');
                    break;
                end
                
                u_temp = zeros(length(t), 2);
                for i = 1:length(t); [~, u_i] = dynamics_flight(t(i), X(i,:)', params); u_temp(i,:) = u_i'; end
                lambda_temp = zeros(length(t), 2);
                
                next_state = 3;
                
            case 3 % Stance phase, stabilizing after landing
                % fprintf('  State 3: Landing Stance from t=%.3f\n', t_start);
                t_stabilize_duration = 2.0;
                t_integrate_end = min(t_start + t_stabilize_duration, t_end);
                
                % --- CRITICAL FIX: Use ode15s (stiff solver) instead of ode45 ---
                [t, X] = ode15s(@(t,x) dynamics_stabilize(t,x,params), [t_start t_integrate_end], x_current);
                
                if isempty(t)
                    % fprintf('ODE solver failed in state 3.\n');
                    break;
                end
                
                u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
                for i = 1:length(t); [~, u_i, lambda_i] = dynamics_stabilize(t(i), X(i,:)', params); u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; end
                
                t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp];
                p_d_y_stance = [p_d_y_stance; ones(size(t)) * params.y_stand];
                p_d_q3_stance = [p_d_q3_stance; zeros(size(t))];
                
                next_state = 99;
        end % end switch
        
        % Store data and transition
        if isempty(t)
            % This happens if the ODE solver fails immediately
            % fprintf('Simulation failed. Breaking loop.\n');
            break;
        else
            % This logic handles appending data correctly
            if ~isempty(t_all)
                % Remove first point to avoid time/state duplication
                if ~isempty(t)
                    t = t(2:end);
                    X = X(2:end, :);
                    u_temp = u_temp(2:end, :);
                    lambda_temp = lambda_temp(2:end, :);
                end
            end
            
            % Only append if there's actually new data
            if ~isempty(t)
                t_all = [t_all; t];
                X_all = [X_all; X];
                u_all = [u_all; u_temp];
                lambda_all = [lambda_all; lambda_temp];
                t_start = t(end);
            else
                % This can happen if the event triggers at the very first step
                % or if the simulation fails to append
                if ~isempty(t_all)
                    t_start = t_all(end);
                else
                    t_start = t_end; % Break loop if t_all is still empty
                end
            end
        end
        
        if next_state == 99 || t_start >= t_end, break; end
        
        % Check if X is empty before trying to access X(end,:)
        if isempty(X)
             % fprintf('State transition failed, X is empty. Breaking.\n');
             break;
        end
        
        x_current = X(end,:)';
        
        if state == 2 
            % fprintf('  Impact at t=%.3f. Recalculating velocities.\n', t_start);
            x_current = dynamics_impact(x_current, params);
            
            % --- START OF FIX ---
            % We just landed. Get the COM x-position *after* impact.
            q_post_impact = x_current(1:5);
            pCOM_landing = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q_post_impact(3),q_post_impact(4),q_post_impact(5),q_post_impact(1),q_post_impact(2))';
            
            % Set the stabilization target to this landing position, not 0.
            % This removes the controller discontinuity.
            params.x_stand_target = pCOM_landing(1);
            % --- END OF FIX ---
        end
        
        state = next_state;
    end % end while loop
    
    % if ~isempty(t_all), fprintf('Simulation finished at t=%.3f\n', t_all(end)); end
    
    % =========================================================================
    % --- 3. Post-Processing (NO PLOTTING) ---
    % =========================================================================
    
    % Check if simulation produced any data at all
    if isempty(t_all)
        % fprintf('Simulation produced no data. Returning empty.\n');
        % Return empty arrays, cost function will handle this
        pCOM_actual = [];
        return;
    end
    
    % fprintf('Processing results...\n');
    pCOM_actual = zeros(length(t_all), 2);
    for i = 1:length(t_all)
       q = X_all(i, 1:5)';
       % Ensure auto_pCOM is on the MATLAB path
       pCOM_actual(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5),q(1),q(2))';
    end

end % --- END OF MAIN FUNCTION run_flip_simulation ---
% =========================================================================
% =========================================================================
% --- LOCAL FUNCTIONS (All code is in this one file) ---
% =========================================================================
% =========================================================================
function [y_d, v_d, a_d, q3_d, q3dot_d, q3ddot_d] = desired_flip_trajectory(t, params)
    % --- 1. Unpack Parameters ---
    T_hold     = params.T_hold; T_jump     = params.T_jump;
    y_stand    = params.y_stand; y_squat    = params.y_squat;
    y_takeoff  = params.y_takeoff; vf_takeoff = params.vf_takeoff;
    g          = 9.81;
    q3_takeoff     = params.q3_takeoff;     
    q3dot_takeoff  = params.q3dot_takeoff;  
    if isfield(params, 'T_dip_ratio'), T_dip_ratio = params.T_dip_ratio;
    else, T_dip_ratio = 0.4; end
    T_dip  = T_jump * T_dip_ratio; T_push = T_jump * (1 - T_dip_ratio);
    t_end_hold   = T_hold; t_end_dip    = T_hold + T_dip; t_end_push   = t_end_dip + T_push;
    % --- 2. Calculate Trajectory for COMy (Unchanged) ---
    if t < t_end_hold
        y_d = y_stand; v_d = 0; a_d = 0;
    elseif t < t_end_dip
        tau_dip = t - t_end_hold;
        A_dip = (y_stand - y_squat) * (2*pi) / (T_dip^2);
        sin_term = sin(2*pi * tau_dip / T_dip); cos_term = cos(2*pi * tau_dip / T_dip);
        a_d = -A_dip * sin_term; v_d = (A_dip * T_dip / (2*pi)) * (cos_term - 1);
        y_d = y_stand + (A_dip * T_dip^2 / (4*pi^2)) * sin_term - (A_dip * T_dip / (2*pi)) * tau_dip;
    elseif t < t_end_push
        tau_push = t - t_end_dip; s = tau_push / T_push;
        P_end = y_takeoff - y_squat; V_end = vf_takeoff * T_push;
        c3 = 10*P_end - 4*V_end; c4 = -15*P_end + 7*V_end; c5 = 6*P_end - 3*V_end;
        s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;
        p_norm = c3*s3 + c4*s4 + c5*s5; v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4; a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;
        y_d = y_squat + p_norm; v_d = v_norm / T_push; a_d = a_norm / (T_push^2);
    else
        tau_flight = t - t_end_push;
        y_d = y_takeoff + vf_takeoff * tau_flight - 0.5 * g * tau_flight^2;
        v_d = vf_takeoff - g * tau_flight; a_d = -g;
    end
    % --- 3. Calculate Trajectory for Torso q3 (The Flip) ---
    t_start_flip = t_end_dip + T_push * params.flip_timing_ratio;
    if t < t_start_flip
        % --- Phase 1, 2a, and most of 2b: HOLD at q3=0 ---
        % Push up vertically first to get height
        q3_d = 0;
        q3dot_d = 0;
        q3ddot_d = 0;
        
    elseif t < t_end_push
        % --- LAST part of Phase 2b: The Flip Kick ---
        % Now, execute a fast polynomial from q3=0 to q3=q3_takeoff
        [q3_d, q3dot_d, q3ddot_d] = quintic_poly(t, t_start_flip, t_end_push, 0, q3_takeoff, 0, q3dot_takeoff, 0, 0);
    else
        % --- Phase 3: Flight (Ballistic Rotation) ---
        tau_flight = t - t_end_push;
        q3_d = q3_takeoff + q3dot_takeoff * tau_flight;
        q3dot_d = q3dot_takeoff;
        q3ddot_d = 0;
    end
end
% ---
% Helper function (this should be at the very end of the file)
% ---
function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
    T = tf - t0;
    if T <= 0, p=pf; v=vf; a=af; return; end
    c0 = p0; c1 = v0; c2 = 0.5 * a0;
    c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
    c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
    c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
    tau = t - t0;
    tau2 = tau*tau; tau3 = tau*tau2; tau4 = tau*tau3; tau5 = tau*tau4;
    p = c0 + c1*tau + c2*tau2 + c3*tau3 + c4*tau4 + c5*tau5;
    v = c1 + 2*c2*tau + 3*c3*tau2 + 4*c4*tau3 + 5*c5*tau4;
    a = 2*c2 + 6*c3*tau + 12*c4*tau2 + 20*c5*tau3;
end
% ---
% STANCE CONTROLLER
% ---
function [dx, u, lambda, p_COMy_d, p_q3_d] = dynamics_stance(t, x, params)
    q = x(1:5); dq = x(6:10);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
    
    Kp = params.Kp; Kd = params.Kd;
    m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g;
    [p_COMy_d, dp_COMy_d, ddp_COMy_d, p_q3_d, dp_q3_d, ddp_q3_d] = desired_flip_trajectory(t, params);
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
    pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2))';
    Jh_com_full = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
    dpCOM_actual = Jh_com_full * dq;
    J_com_dot_full = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));
    Jh_task = [ Jh_com_full(2, :); 0 0 0 0 1 ];
    J_task_dot = [ J_com_dot_full(2, :); 0 0 0 0 0 ];
    
    p_tilde = [ pCOM_actual(2) - p_COMy_d; q(5) - p_q3_d ];
    dp_tilde = [ dpCOM_actual(2) - dp_COMy_d; dq(5) - dp_q3_d ]; 
    v = [ ddp_COMy_d; ddp_q3_d ] - Kp*p_tilde - Kd*dp_tilde;
    q1_min = 0.5; q2_max = 2.9;
    K_limit = 1000; D_limit = 100;
    tau_penalty = [0; 0];
    if q1 < q1_min, tau_penalty(1) = K_limit * (q1_min - q1) - D_limit * q1dot; end
    if q2 > q2_max, tau_penalty(2) = K_limit * (q2_max - q2) - D_limit * q2dot; end
    n_q = 5; n_u = 2; n_c_task = 2; n_c_constr = 2;
    LHS = [ D, -B, -Jst';
            Jh_task, zeros(n_c_task, n_u), zeros(n_c_task, n_c_constr);
            Jst, zeros(n_c_constr, n_u), zeros(n_c_constr, n_c_constr) ];
    RHS = [ -C*dq - G + B*tau_penalty;
            v - J_task_dot*dq;
            -Jstdot*dq ];
    LHS = LHS + eye(size(LHS)) * 1e-6; 
    
    sol = LHS \ RHS;
    ddq = sol(1:5); u = sol(6:7); lambda = sol(8:9);
    dx = [dq; ddq];
end
% ---
% FLIGHT DYNAMICS
% ---
function [dx, u] = dynamics_flight(~, x, params)
    q  = x(1:5); dq = x(6:10);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
    m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g;
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    % --- TUCK CONTROLLER ---
    if isfield(params,'q_tuck_des')
        qd = params.q_tuck_des(:);
    else
        qd = [q1; q2]; % fallback
    end
    Kp = params.flight_Kp; Kd = params.flight_Kd; umax = params.flight_u_max;
    err = qd - [q1; q2];
    derr = - [q1dot; q2dot];
    u_raw = Kp .* err + Kd .* derr;
    u = max(min(u_raw, umax(:)), -umax(:));
    % ---
    
    ddq = D \ (-C*dq - G + B*u);
    dx = [dq; ddq];
end
% ---
% IMPACT DYNAMICS
% ---
function x_plus = dynamics_impact(x_minus, params)
    q = x_minus(1:5); dq_minus = x_minus(6:10);
    m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
    q1=q(3); q2=q(4); q3=q(5);
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    Jst = auto_Jst(l1,l2,q1,q2); 
    n_q = 5; n_c = 2;
    LHS = [ D,   -Jst'; Jst,  zeros(n_c, n_c) ];
    RHS = [ D * dq_minus; zeros(n_c, 1) ];
    
    solution = LHS \ RHS;
    dq_plus = solution(1:n_q);
    x_plus = [q; dq_plus];
end
% ---
% STABILIZE CONTROLLER
% ---
function [dx, u, lambda] = dynamics_stabilize(~, x, params)
    % This is the original simple stabilizer from leg_ode.m
    q  = x(1:5); dq = x(6:10);
    x_hip = q(1); y_hip = q(2); q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
    
    m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g; pCOMy_d = params.y_stand;
    Kp=params.Kp; Kd=params.Kd;
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
    % Simple PD on COMx=0, COMy=y_stand
    pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2))';
    Jh_com = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
    dpCOM_actual = Jh_com * dq;
    J_com_dot = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));
    
    % --- START OF FIX ---
    % Instead of hard-coding a target of x=0, use the x_stand_target
    % that we set upon impact.
    if isfield(params, 'x_stand_target')
        pCOMx_d = params.x_stand_target; % Target the x-pos we landed at
    else
        pCOMx_d = 0; % Fallback just in case
    end
    
    p_tilde = [pCOM_actual(1) - pCOMx_d; pCOM_actual(2) - pCOMy_d];
    % --- END OF FIX ---
    
    dp_tilde = [dpCOM_actual(1) - 0; dpCOM_actual(2) - 0];
    v = [0; 0] - Kp*p_tilde - Kd*dp_tilde;
    
    n_q = 5; n_u = 2; n_c = 2;
    LHS = [ D,      -B,             -Jst';
            Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
            Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];
    RHS = [ -C*dq - G;
            v - J_com_dot*dq;
            -Jstdot*dq ];
    
    sol = LHS \ RHS;
    ddq = sol(1:5); u = sol(6:7); lambda = sol(8:9);
    dx = [dq; ddq];
end
% ---
% EVENT FUNCTIONS
% ---
function [value, isterminal, direction] = event_takeoff(t, x, params)
    [~, ~, lambda] = dynamics_stance(t, x, params);
    Fy = lambda(2);
    
    % Robustness: Don't take off if COM is moving down
    q = x(1:5); dq = x(6:10);
    pCOM_vel = auto_JpCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5)) * dq;
    
    % Check if pCOM_vel is not empty
    if isempty(pCOM_vel) || pCOM_vel(2) < 0.1 % Must be moving up
        value = 1; % Stay positive, don't trigger
        isterminal = 0;
        direction = 0;
        return;
    end
    
    value = Fy;          
    isterminal = 1;      
    direction = -1;      
end

function [value, isterminal, direction] = event_touchdown(~, x, params)
    q = x(1:5);
    pFoot = auto_pfoot(params.l1, params.l2, q(3), q(4), q(1), q(2));
    yFoot = pFoot(2);
    value = yFoot;
    isterminal = 1;
    direction = -1;
end

% ---
% INITIAL STATE CALCULATOR
% ---
function x0 = calculate_stable_initial_state(params)
    % fprintf('Calculating a stable initial state...\n');
    ik_errors = @(vars) ik_stability_helper(vars, params);
    initial_guess = [1.3; 1.3; 0.0; params.y_stand_target]; 
    options = optimoptions('fsolve', 'Display', 'none');
    solution = fsolve(ik_errors, initial_guess, options);
    q1_sol = solution(1); q2_sol = solution(2);
    hip_x_sol = solution(3); hip_y_sol = solution(4);
    x0 = [hip_x_sol; hip_y_sol; q1_sol; q2_sol; 0.0; 0; 0; 0; 0; 0;];
    % fprintf('  -> Stable state found: y_hip=%.4f, x_hip=%.4f, q1=%.3f, q2=%.3f\n', x0(2), x0(1), x0(3), x0(4));
end
function errors = ik_stability_helper(vars, params)
    q1 = vars(1); q2 = vars(2); hip_x = vars(3); hip_y = vars(4);
    pFoot = auto_pfoot(params.l1, params.l2, q1, q2, hip_x, hip_y);
    % CRITICAL: Use d3 in the IK solver
    pCOM  = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,0,hip_x,hip_y);
    errors = [pFoot(2); pFoot(1); pCOM(1); pCOM(2) - params.y_stand_target];
end

% function [t_all, X_all, u_all, pCOM_actual] = run_flip_simulation(params)
%     % -------------------------------------------------------------------------
%     % This function runs a single simulation for the flip controller
%     % given a set of parameters. It is designed to be called by an
%     % optimizer and returns the simulation results for cost calculation.
%     % It contains NO plotting or animation.
%     %
%     % It uses the STIFF solver 'ode15s' for stability.
%     % -------------------------------------------------------------------------
% 
%     % --- Initial State ---
%     % We assume 'auto_...' functions are on the path
%     try
%         x0 = calculate_stable_initial_state(params);
%     catch ME
%         % If IK solver fails, this is a bad set of params
%         fprintf('  calculate_stable_initial_state failed: %s\n', ME.message);
%         t_all = []; X_all = []; u_all = []; pCOM_actual = [];
%         return;
%     end
% 
%     % =========================================================================
%     % --- 2. Event-Driven Simulation Loop ---
%     % =========================================================================
%     t_start = 0;
%     t_end = 8;
%     x_current = x0;
% 
%     % Initialize data storage (CRITICAL to prevent 't_all' error)
%     t_all = []; X_all = []; u_all = []; lambda_all = [];
%     t_stance = []; lambda_stance = []; p_d_y_stance = []; p_d_q3_stance = [];
% 
%     % Setup ODE options
%     opts_stance = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_takeoff(t,x,params));
%     opts_flight = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_touchdown(t,x,params), 'MaxStep', 1e-2);
% 
%     state = 1;
% 
%     % fprintf('Starting STABLE flip simulation (tracking q3)...\n');
%     while t_start < t_end
%         switch state
%             case 1 % Stance phase, executing the flip
%                 % fprintf('  State 1: Flipping Stance from t=%.3f\n', t_start);
% 
%                 % --- CRITICAL FIX: Use ode15s (stiff solver) instead of ode45 ---
%                 [t, X, ~, ~, ~] = ode15s(@(t,x) dynamics_stance(t,x,params), [t_start t_end], x_current, opts_stance);
% 
%                 % Check for immediate failure from ode15s
%                 if isempty(t)
%                     % fprintf('ODE solver failed in state 1.\n');
%                     break; % Exit switch and let loop handle empty 't'
%                 end
% 
%                 u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
%                 p_d_y_temp = zeros(length(t),1); p_d_q3_temp = zeros(length(t),1);
% 
%                 for i = 1:length(t)
%                     % Recalculate dynamics to get u and lambda
%                     [~, u_i, lambda_i, p_d_y, p_d_q3] = dynamics_stance(t(i), X(i,:)', params);
%                     u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i';
%                     p_d_y_temp(i) = p_d_y; p_d_q3_temp(i) = p_d_q3;
%                 end
%                 t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp];
%                 p_d_y_stance = [p_d_y_stance; p_d_y_temp]; p_d_q3_stance = [p_d_q3_stance; p_d_q3_temp];
% 
%                 next_state = 2;
%                 params.q_des_flight = X(end,3:4);
% 
%             case 2 % Flight phase
%                 % fprintf('  State 2: Flight from t=%.3f\n', t_start);
% 
%                 % --- Use ode15s here too for consistency ---
%                 [t, X, ~, ~, ~] = ode15s(@(t,x) dynamics_flight(t,x,params), [t_start t_end], x_current, opts_flight);
% 
%                 if isempty(t)
%                     % fprintf('ODE solver failed in state 2.\n');
%                     break;
%                 end
% 
%                 u_temp = zeros(length(t), 2);
%                 for i = 1:length(t); [~, u_i] = dynamics_flight(t(i), X(i,:)', params); u_temp(i,:) = u_i'; end
%                 lambda_temp = zeros(length(t), 2);
% 
%                 next_state = 3;
% 
%             case 3 % Stance phase, stabilizing after landing
%                 % fprintf('  State 3: Landing Stance from t=%.3f\n', t_start);
%                 t_stabilize_duration = 2.0;
%                 t_integrate_end = min(t_start + t_stabilize_duration, t_end);
% 
%                 % --- CRITICAL FIX: Use ode15s (stiff solver) instead of ode45 ---
%                 [t, X] = ode15s(@(t,x) dynamics_stabilize(t,x,params), [t_start t_integrate_end], x_current);
% 
%                 if isempty(t)
%                     % fprintf('ODE solver failed in state 3.\n');
%                     break;
%                 end
% 
%                 u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
%                 for i = 1:length(t); [~, u_i, lambda_i] = dynamics_stabilize(t(i), X(i,:)', params); u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; end
% 
%                 t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp];
%                 p_d_y_stance = [p_d_y_stance; ones(size(t)) * params.y_stand];
%                 p_d_q3_stance = [p_d_q3_stance; zeros(size(t))];
% 
%                 next_state = 99;
%         end % end switch
% 
%         % Store data and transition
%         if isempty(t)
%             % This happens if the ODE solver fails immediately
%             % fprintf('Simulation failed. Breaking loop.\n');
%             break;
%         else
%             % This logic handles appending data correctly
%             if ~isempty(t_all)
%                 % Remove first point to avoid time/state duplication
%                 t = t(2:end);
%                 X = X(2:end, :);
%                 u_temp = u_temp(2:end, :);
%                 lambda_temp = lambda_temp(2:end, :);
%             end
% 
%             % Only append if there's actually new data
%             if ~isempty(t)
%                 t_all = [t_all; t];
%                 X_all = [X_all; X];
%                 u_all = [u_all; u_temp];
%                 lambda_all = [lambda_all; lambda_temp];
%                 t_start = t(end);
%             else
%                 % This can happen if the event triggers at the very first step
%                 % Avoids an infinite loop
%                 t_start = t_end;
%             end
%         end
% 
%         if next_state == 99 || t_start >= t_end, break; end
% 
%         x_current = X(end,:)';
% 
%         if state == 2
%             % fprintf('  Impact at t=%.3f. Recalculating velocities.\n', t_start);
%             x_current = dynamics_impact(x_current, params);
%         end
% 
%         state = next_state;
%     end % end while loop
% 
%     % if ~isempty(t_all), fprintf('Simulation finished at t=%.3f\n', t_all(end)); end
% 
%     % =========================================================================
%     % --- 3. Post-Processing (NO PLOTTING) ---
%     % =========================================================================
% 
%     % Check if simulation produced any data at all
%     if isempty(t_all)
%         % fprintf('Simulation produced no data. Returning empty.\n');
%         % Return empty arrays, cost function will handle this
%         pCOM_actual = [];
%         return;
%     end
% 
%     % fprintf('Processing results...\n');
%     pCOM_actual = zeros(length(t_all), 2);
%     for i = 1:length(t_all)
%        q = X_all(i, 1:5)';
%        % Ensure auto_pCOM is on the MATLAB path
%        pCOM_actual(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5),q(1),q(2))';
%     end
% 
% end % --- END OF MAIN FUNCTION run_flip_simulation ---
% 
% % function test_flip_controller()
% %     % -------------------------------------------------------------------------
% %     % This is a single, self-contained file to test a STABLE flip controller.
% %     %
% %     % IT CONTROLS:
% %     %   1. Vertical Height (COMy)
% %     %   2. Torso Angle (q3)
% %     %
% %     % This approach is STABLE because the controller's goals do not
% %     % conflict with the physics.
% %     % -------------------------------------------------------------------------
% % 
% %     clear all; close all; clc;
% % 
% %     % Add the path to your 'auto_...' functions if they are in a subfolder
% %     % addpath('../auto/'); 
% % 
% %     % =========================================================================
% %     % --- 1. Simulation and Model Parameters ---
% %     % =========================================================================
% %     params.m1 = 0.02;   params.m2 = 0.02;   params.m3 = 0.3;
% %     params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
% %     params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.05;
% % 
% %     params.g  = 9.81;
% %     params.I1 = (1/12) * params.m1 * params.l1^2;
% %     params.I2 = (1/12) * params.m2 * params.l2^2;
% %     params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);
% % 
% %     % --- Trajectory Parameters ---
% %     params.y_stand_target = 0.22;
% %     params.y_stand        = params.y_stand_target;
% %     params.T_jump         = 1.0;  % Give it time
% %     params.y_squat        = 0.10; % Shallow, stable squat
% %     params.y_takeoff      = 0.24;
% %     params.T_hold         = 1.0;
% %     params.vf_takeoff     = 0.5;
% %     params.T_dip_ratio    = 0.5;  % 50% dip, 50% push
% % 
% %     % --- FLIP TUNING PARAMETERS ---
% %     % This is what you will tune. Start small.
% %     params.q3_takeoff     = deg2rad(-0.05);  % (90 deg)
% %     params.q3dot_takeoff  = 6;     % (rad/s)
% %     params.flip_timing_ratio = 0.45;
% % 
% %     % --- CONTROLLER GAINS ---
% %     % These gains are for the [COMy, q3] controller.
% %     params.Kp = 50;
% %     params.Kd = 5;
% % 
% %     % Flight gains (for tucking)
% %     params.flight_Kp = [50; 50];
% %     params.flight_Kd = [5; 5];
% %     params.flight_u_max = [2.0; 2.0];
% %     params.q_tuck_des = [deg2rad(270); deg2rad(170)]; % Tuck angles
% % 
% %     % --- Initial State ---
% %     x0 = calculate_stable_initial_state(params);
% % 
% %     % =========================================================================
% %     % --- 2. Event-Driven Simulation Loop ---
% %     % =========================================================================
% %     t_start = 0;
% %     t_end = 8;
% %     x_current = x0;
% % 
% %     t_all = []; X_all = []; u_all = []; lambda_all = [];
% %     t_stance = []; lambda_stance = []; p_d_y_stance = []; p_d_q3_stance = [];
% % 
% %     opts_stance = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_takeoff(t,x,params));
% %     opts_flight = odeset('RelTol',1e-6,'AbsTol',1e-6, 'Events', @(t,x) event_touchdown(t,x,params), 'MaxStep', 1e-2);
% % 
% %     state = 1; 
% % 
% %     fprintf('Starting STABLE flip simulation (tracking q3)...\n');
% %     while t_start < t_end
% %         switch state
% %             case 1 % Stance phase, executing the flip
% %                 fprintf('  State 1: Flipping Stance from t=%.3f\n', t_start);
% %                 [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_stance(t,x,params), [t_start t_end], x_current, opts_stance);
% % 
% %                 u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
% %                 p_d_y_temp = zeros(length(t),1); p_d_q3_temp = zeros(length(t),1);
% % 
% %                 for i = 1:length(t)
% %                     [~, u_i, lambda_i, p_d_y, p_d_q3] = dynamics_stance(t(i), X(i,:)', params); 
% %                     u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; 
% %                     p_d_y_temp(i) = p_d_y; p_d_q3_temp(i) = p_d_q3;
% %                 end
% %                 t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp]; 
% %                 p_d_y_stance = [p_d_y_stance; p_d_y_temp]; p_d_q3_stance = [p_d_q3_stance; p_d_q3_temp];
% % 
% %                 next_state = 2;
% %                 params.q_des_flight = X(end,3:4);
% % 
% %             case 2 % Flight phase
% %                 fprintf('  State 2: Flight from t=%.3f\n', t_start);
% %                 [t, X, ~, ~, ~] = ode45(@(t,x) dynamics_flight(t,x,params), [t_start t_end], x_current, opts_flight);
% % 
% %                 u_temp = zeros(length(t), 2);
% %                 for i = 1:length(t); [~, u_i] = dynamics_flight(t(i), X(i,:)', params); u_temp(i,:) = u_i'; end
% %                 lambda_temp = zeros(length(t), 2);
% % 
% %                 next_state = 3;
% % 
% %             case 3 % Stance phase, stabilizing after landing
% %                 fprintf('  State 3: Landing Stance from t=%.3f\n', t_start);
% %                 t_stabilize_duration = 2.0;
% %                 t_integrate_end = min(t_start + t_stabilize_duration, t_end);
% % 
% %                 [t, X] = ode45(@(t,x) dynamics_stabilize(t,x,params), [t_start t_integrate_end], x_current);
% % 
% %                 u_temp = zeros(length(t), 2); lambda_temp = zeros(length(t), 2);
% %                 for i = 1:length(t); [~, u_i, lambda_i] = dynamics_stabilize(t(i), X(i,:)', params); u_temp(i,:) = u_i'; lambda_temp(i,:) = lambda_i'; end
% % 
% %                 t_stance = [t_stance; t]; lambda_stance = [lambda_stance; lambda_temp]; 
% %                 p_d_y_stance = [p_d_y_stance; ones(size(t)) * params.y_stand];
% %                 p_d_q3_stance = [p_d_q3_stance; zeros(size(t))]; 
% % 
% %                 next_state = 99;
% %         end
% % 
% %         % Store data and transition
% %         if isempty(t)
% %             t_start = t_end; 
% %         else
% %             if ~isempty(t_all)
% %                 t = t(2:end); X = X(2:end, :); u_temp = u_temp(2:end, :); lambda_temp = lambda_temp(2:end, :);
% %             end
% %             t_all = [t_all; t]; X_all = [X_all; X]; u_all = [u_all; u_temp]; lambda_all = [lambda_all; lambda_temp];
% %             t_start = t(end);
% %         end
% % 
% %         if next_state == 99 || t_start >= t_end, break; end
% % 
% %         x_current = X(end,:)';
% % 
% %         if state == 2 
% %             fprintf('  Impact at t=%.3f. Recalculating velocities.\n', t_start);
% %             x_current = dynamics_impact(x_current, params);
% %         end
% % 
% %         state = next_state;
% %     end
% %     fprintf('Simulation finished at t=%.3f\n', t_all(end));
% % 
% % 
% %     % =========================================================================
% %     % --- 3. Post-Processing and Plotting ---
% %     % =========================================================================
% %     fprintf('Processing and plotting results...\n');
% % 
% %     pCOM_actual = zeros(length(t_all), 2);
% %     for i = 1:length(t_all)
% %        q = X_all(i, 1:5)';
% %        pCOM_actual(i,:) = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5),q(1),q(2))';
% %     end
% % 
% %     % --- FIGURE 1: VISUALIZE THE CONTROLLER ---
% %     % This is what you asked for: a plot showing the controller
% %     % tracking the angle from 0 to the target.
% %     figure('Name', 'Flip Controller Tracking');
% %     subplot(2,1,1);
% %     plot(t_stance, p_d_y_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired COM-Y'); hold on;
% %     plot(t_all, pCOM_actual(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-Y');
% %     grid on; title('Center of Mass Height vs. Time'); xlabel('Time (s)'); ylabel('Height (m)'); legend;
% % 
% %     subplot(2,1,2);
% %     plot(t_stance, p_d_q3_stance, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Desired Torso q3'); hold on;
% %     plot(t_all, X_all(:,5), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Torso q3');
% %     grid on; title('Torso Angle (q3) vs. Time'); xlabel('Time (s)'); ylabel('Angle (rad)'); legend;
% % 
% %     % --- FIGURE 2: See the "cost" of the flip ---
% %     % This plot will show the horizontal drift (COMx) and the
% %     % horizontal force (Fx) that the controller *chose* to use.
% %     figure('Name', 'Uncontrolled Dynamics and Forces');
% %     subplot(2,1,1);
% %     plot(t_all, pCOM_actual(:,1), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual COM-X (Drift)');
% %     grid on; title('Horizontal COM Drift (This is what causes the flip!)'); xlabel('Time (s)'); ylabel('Position (m)'); legend;
% %     subplot(2,1,2);
% %     plot(t_all, lambda_all(:, 1), 'r-', 'LineWidth', 2, 'DisplayName', 'Force Fx');
% %     grid on; title('Horizontal Ground Force (Fx)'); xlabel('Time (s)'); ylabel('Force (N)'); legend;
% % 
% %     % --- FIGURE 3: ANIMATION ---
% %     fprintf('Generating animation... (this may take a moment)\n');
% %     % This uses the animate_flipping_gif.m file you uploaded.
% %     % If it's not on your path, this line will error.
% %     try
% %         animate_flipping_gif(t_all, X_all, params, 'leg_flip_animation.gif');
% %     catch
% %         fprintf('Could not generate animation. Make sure animate_flipping_gif.m is on your path.\n');
% %     end
% % end
% 
% % =========================================================================
% % =========================================================================
% % --- LOCAL FUNCTIONS (All code is in this one file) ---
% % =========================================================================
% % =========================================================================
% 
% function [y_d, v_d, a_d, q3_d, q3dot_d, q3ddot_d] = desired_flip_trajectory(t, params)
%     % --- 1. Unpack Parameters ---
%     T_hold     = params.T_hold; T_jump     = params.T_jump;
%     y_stand    = params.y_stand; y_squat    = params.y_squat;
%     y_takeoff  = params.y_takeoff; vf_takeoff = params.vf_takeoff;
%     g          = 9.81;
%     q3_takeoff     = params.q3_takeoff;     
%     q3dot_takeoff  = params.q3dot_takeoff;  
%     if isfield(params, 'T_dip_ratio'), T_dip_ratio = params.T_dip_ratio;
%     else, T_dip_ratio = 0.4; end
%     T_dip  = T_jump * T_dip_ratio; T_push = T_jump * (1 - T_dip_ratio);
%     t_end_hold   = T_hold; t_end_dip    = T_hold + T_dip; t_end_push   = t_end_dip + T_push;
% 
%     % --- 2. Calculate Trajectory for COMy (Unchanged) ---
%     if t < t_end_hold
%         y_d = y_stand; v_d = 0; a_d = 0;
%     elseif t < t_end_dip
%         tau_dip = t - t_end_hold;
%         A_dip = (y_stand - y_squat) * (2*pi) / (T_dip^2);
%         sin_term = sin(2*pi * tau_dip / T_dip); cos_term = cos(2*pi * tau_dip / T_dip);
%         a_d = -A_dip * sin_term; v_d = (A_dip * T_dip / (2*pi)) * (cos_term - 1);
%         y_d = y_stand + (A_dip * T_dip^2 / (4*pi^2)) * sin_term - (A_dip * T_dip / (2*pi)) * tau_dip;
%     elseif t < t_end_push
%         tau_push = t - t_end_dip; s = tau_push / T_push;
%         P_end = y_takeoff - y_squat; V_end = vf_takeoff * T_push;
%         c3 = 10*P_end - 4*V_end; c4 = -15*P_end + 7*V_end; c5 = 6*P_end - 3*V_end;
%         s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;
%         p_norm = c3*s3 + c4*s4 + c5*s5; v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4; a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;
%         y_d = y_squat + p_norm; v_d = v_norm / T_push; a_d = a_norm / (T_push^2);
%     else
%         tau_flight = t - t_end_push;
%         y_d = y_takeoff + vf_takeoff * tau_flight - 0.5 * g * tau_flight^2;
%         v_d = vf_takeoff - g * tau_flight; a_d = -g;
%     end
% 
%     % --- 3. Calculate Trajectory for Torso q3 (The Flip) ---
%     t_start_flip = t_end_dip + T_push * params.flip_timing_ratio;
% 
%     if t < t_start_flip
%         % --- Phase 1, 2a, and most of 2b: HOLD at q3=0 ---
%         % Push up vertically first to get height
%         q3_d = 0;
%         q3dot_d = 0;
%         q3ddot_d = 0;
% 
%     elseif t < t_end_push
%         % --- LAST part of Phase 2b: The Flip Kick ---
%         % Now, execute a fast polynomial from q3=0 to q3=q3_takeoff
%         [q3_d, q3dot_d, q3ddot_d] = quintic_poly(t, t_start_flip, t_end_push, 0, q3_takeoff, 0, q3dot_takeoff, 0, 0);
% 
%     else
%         % --- Phase 3: Flight (Ballistic Rotation) ---
%         tau_flight = t - t_end_push;
%         q3_d = q3_takeoff + q3dot_takeoff * tau_flight;
%         q3dot_d = q3dot_takeoff;
%         q3ddot_d = 0;
%     end
% 
%     % if t < t_end_dip
%     %     q3_d = 0; q3dot_d = 0; q3ddot_d = 0;
%     % elseif t < t_end_push
%     %     tau_push = t - t_end_dip; s = tau_push / T_push;
%     %     P_end = q3_takeoff; V_end = q3dot_takeoff * T_push;
%     %     c3 = 10*P_end - 4*V_end; c4 = -15*P_end + 7*V_end; c5 = 6*P_end - 3*V_end;
%     %     s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;
%     %     p_norm = c3*s3 + c4*s4 + c5*s5; v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4; a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;
%     %     q3_d = p_norm; q3dot_d = v_norm / T_push; q3ddot_d = a_norm / (T_push^2);
%     % else
%     %     tau_flight = t - t_end_push;
%     %     q3_d = q3_takeoff + q3dot_takeoff * tau_flight;
%     %     q3dot_d = q3dot_takeoff; q3ddot_d = 0;
%     % end
% end
% 
% % ---
% % Helper function (this should be at the very end of the file)
% % ---
% function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
%     T = tf - t0;
%     if T <= 0, p=pf; v=vf; a=af; return; end
%     c0 = p0; c1 = v0; c2 = 0.5 * a0;
%     c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
%     c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
%     c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
%     tau = t - t0;
%     tau2 = tau*tau; tau3 = tau*tau2; tau4 = tau*tau3; tau5 = tau*tau4;
%     p = c0 + c1*tau + c2*tau2 + c3*tau3 + c4*tau4 + c5*tau5;
%     v = c1 + 2*c2*tau + 3*c3*tau2 + 4*c4*tau3 + 5*c5*tau4;
%     a = 2*c2 + 6*c3*tau + 12*c4*tau2 + 20*c5*tau3;
% end
% 
% % ---
% % STANCE CONTROLLER
% % ---
% function [dx, u, lambda, p_COMy_d, p_q3_d] = dynamics_stance(t, x, params)
%     q = x(1:5); dq = x(6:10);
%     q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     Kp = params.Kp; Kd = params.Kd;
%     m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
%     g = params.g;
% 
%     [p_COMy_d, dp_COMy_d, ddp_COMy_d, p_q3_d, dp_q3_d, ddp_q3_d] = desired_flip_trajectory(t, params);
% 
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
%     Jst = auto_Jst(l1,l2,q1,q2);
%     Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
% 
%     pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2));
%     Jh_com_full = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
%     dpCOM_actual = Jh_com_full * dq;
%     J_com_dot_full = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));
% 
%     Jh_task = [ Jh_com_full(2, :); 0 0 0 0 1 ];
%     J_task_dot = [ J_com_dot_full(2, :); 0 0 0 0 0 ];
% 
%     p_tilde = [ pCOM_actual(2) - p_COMy_d; q(5) - p_q3_d ];
%     dp_tilde = [ dpCOM_actual(2) - dp_COMy_d; dq(5) - dp_q3_d ]; 
%     v = [ ddp_COMy_d; ddp_q3_d ] - Kp*p_tilde - Kd*dp_tilde;
% 
%     q1_min = 0.5; q2_max = 2.9;
%     K_limit = 1000; D_limit = 100;
%     tau_penalty = [0; 0];
%     if q1 < q1_min, tau_penalty(1) = K_limit * (q1_min - q1) - D_limit * q1dot; end
%     if q2 > q2_max, tau_penalty(2) = K_limit * (q2_max - q2) - D_limit * q2dot; end
% 
%     n_q = 5; n_u = 2; n_c_task = 2; n_c_constr = 2;
%     LHS = [ D, -B, -Jst';
%             Jh_task, zeros(n_c_task, n_u), zeros(n_c_task, n_c_constr);
%             Jst, zeros(n_c_constr, n_u), zeros(n_c_constr, n_c_constr) ];
%     RHS = [ -C*dq - G + B*tau_penalty;
%             v - J_task_dot*dq;
%             -Jstdot*dq ];
%     LHS = LHS + eye(size(LHS)) * 1e-6; 
% 
%     sol = LHS \ RHS;
%     ddq = sol(1:5); u = sol(6:7); lambda = sol(8:9);
%     dx = [dq; ddq];
% end
% 
% % ---
% % FLIGHT DYNAMICS
% % ---
% function [dx, u] = dynamics_flight(~, x, params)
%     q  = x(1:5); dq = x(6:10);
%     q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
%     g = params.g;
% 
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
% 
%     % --- TUCK CONTROLLER ---
%     if isfield(params,'q_tuck_des')
%         qd = params.q_tuck_des(:);
%     else
%         qd = [q1; q2]; % fallback
%     end
%     Kp = params.flight_Kp; Kd = params.flight_Kd; umax = params.flight_u_max;
%     err = qd - [q1; q2];
%     derr = - [q1dot; q2dot];
%     u_raw = Kp .* err + Kd .* derr;
%     u = max(min(u_raw, umax(:)), -umax(:));
%     % ---
% 
%     ddq = D \ (-C*dq - G + B*u);
%     dx = [dq; ddq];
% end
% 
% % ---
% % IMPACT DYNAMICS
% % ---
% function x_plus = dynamics_impact(x_minus, params)
%     q = x_minus(1:5); dq_minus = x_minus(6:10);
%     m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
%     q1=q(3); q2=q(4); q3=q(5);
% 
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     Jst = auto_Jst(l1,l2,q1,q2); 
% 
%     n_q = 5; n_c = 2;
%     LHS = [ D,   -Jst'; Jst,  zeros(n_c, n_c) ];
%     RHS = [ D * dq_minus; zeros(n_c, 1) ];
% 
%     solution = LHS \ RHS;
%     dq_plus = solution(1:n_q);
%     x_plus = [q; dq_plus];
% end
% 
% % ---
% % STABILIZE CONTROLLER
% % ---
% function [dx, u, lambda] = dynamics_stabilize(~, x, params)
%     % This is the original simple stabilizer from leg_ode.m
%     q  = x(1:5); dq = x(6:10);
%     x_hip = q(1); y_hip = q(2); q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     m1=params.m1; m2=params.m2; m3=params.m3; I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3; d1=params.d1; d2=params.d2; d3=params.d3;
%     g = params.g; pCOMy_d = params.y_stand;
%     Kp=params.Kp; Kd=params.Kd;
% 
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
%     Jst = auto_Jst(l1,l2,q1,q2);
%     Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
% 
%     % Simple PD on COMx=0, COMy=y_stand
%     pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2));
%     Jh_com = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
%     dpCOM_actual = Jh_com * dq;
%     J_com_dot = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));
% 
%     p_tilde = [pCOM_actual(1) - 0; pCOM_actual(2) - pCOMy_d];
%     dp_tilde = [dpCOM_actual(1) - 0; dpCOM_actual(2) - 0];
%     v = [0; 0] - Kp*p_tilde - Kd*dp_tilde;
% 
%     n_q = 5; n_u = 2; n_c = 2;
%     LHS = [ D,      -B,             -Jst';
%             Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
%             Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];
%     RHS = [ -C*dq - G;
%             v - J_com_dot*dq;
%             -Jstdot*dq ];
% 
%     sol = LHS \ RHS;
%     ddq = sol(1:5); u = sol(6:7); lambda = sol(8:9);
%     dx = [dq; ddq];
% end
% 
% % ---
% % EVENT FUNCTIONS
% % ---
% function [value, isterminal, direction] = event_takeoff(t, x, params)
%     [~, ~, lambda] = dynamics_stance(t, x, params);
%     Fy = lambda(2);
% 
%     % Robustness: Don't take off if COM is moving down
%     q = x(1:5); dq = x(6:10);
%     pCOM_vel = auto_JpCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q(3),q(4),q(5)) * dq;
%     if pCOM_vel(2) < 0.1 % Must be moving up
%         value = 1; % Stay positive, don't trigger
%         isterminal = 0;
%         direction = 0;
%         return;
%     end
% 
%     value = Fy;          
%     isterminal = 1;      
%     direction = -1;      
% end
% 
% function [value, isterminal, direction] = event_touchdown(~, x, params)
%     q = x(1:5);
%     pFoot = auto_pfoot(params.l1, params.l2, q(3), q(4), q(1), q(2));
%     yFoot = pFoot(2);
%     value = yFoot;
%     isterminal = 1;
%     direction = -1;
% end
% 
% % ---
% % INITIAL STATE CALCULATOR
% % ---
% function x0 = calculate_stable_initial_state(params)
%     fprintf('Calculating a stable initial state...\n');
%     ik_errors = @(vars) ik_stability_helper(vars, params);
%     initial_guess = [1.3; 1.3; 0.0; params.y_stand_target]; 
%     options = optimoptions('fsolve', 'Display', 'none');
%     solution = fsolve(ik_errors, initial_guess, options);
%     q1_sol = solution(1); q2_sol = solution(2);
%     hip_x_sol = solution(3); hip_y_sol = solution(4);
%     x0 = [hip_x_sol; hip_y_sol; q1_sol; q2_sol; 0.0; 0; 0; 0; 0; 0;];
%     fprintf('  -> Stable state found: y_hip=%.4f, x_hip=%.4f, q1=%.3f, q2=%.3f\n', x0(2), x0(1), x0(3), x0(4));
% end
% 
% function errors = ik_stability_helper(vars, params)
%     q1 = vars(1); q2 = vars(2); hip_x = vars(3); hip_y = vars(4);
%     pFoot = auto_pfoot(params.l1, params.l2, q1, q2, hip_x, hip_y);
%     % CRITICAL: Use d3 in the IK solver
%     pCOM  = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,0,hip_x,hip_y);
%     errors = [pFoot(2); pFoot(1); pCOM(1); pCOM(2) - params.y_stand_target];
% end