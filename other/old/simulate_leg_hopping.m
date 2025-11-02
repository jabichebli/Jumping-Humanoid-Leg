
% This code makes it hop 10 times using the simulate_leg_jumping.m code 

function simulate_leg_hopping()

    clear; close all; clc;

    % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.0; 
    params.g  = 9.81; 
    params.q1d = 0; params.q2d = 0;

    % Moments of inertia
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);

    params.m_t = params.m1 + params.m2 + params.m3;

    % Trajectory Parameters
    params.y_stand   = 0.2;
    params.y_squat   = 0.08;
    params.y_takeoff = 0.65;
    params.T_hold    = 2.0;
    params.T_squat   = 0.6;
    params.T_push    = 0.3;

    % Virtual constraint gains
    params.Kp = 30;
    params.Kd = 8;

    % Flight PD and torque limits
    params.flight_Kp = [10; 10]; 
    params.flight_Kd = [2; 2];
    params.flight_u_max = [2; 2];  % N*m saturation


    % Initial condition (standing)
    x0 = [0; 0.2970; 0.2; 0.2; 0; 0; 0; 0; 0; 0];

    % ================= MAIN LOOP =================
    N_hops = 10;  % number of hops
    t_all = []; X_all = []; u_all = [];

    for hop = 1:N_hops
        fprintf('\n================ Hop %d ================\n', hop);

        % ----------------- Stance Phase -----------------
        tspan_stance = [0 10];
        stance_options = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01);
        [t_stance, X_stance] = ode45(@(t,x) dynamics_stance(t,x,params), tspan_stance, x0, stance_options);

        % Extract COM height and find takeoff
        yCOM_stance = zeros(length(t_stance),1);
        for i = 1:length(t_stance)
            [~, ~, ~, yCOM] = dynamics_stance(t_stance(i), X_stance(i,:).', params);
            yCOM_stance(i) = yCOM;
        end
        takeoff_idx = find(yCOM_stance > params.y_takeoff, 1);
        if isempty(takeoff_idx)
            takeoff_idx = length(t_stance);
            disp("Not enough lift for takeoff.");
        end

        % Crop stance data up to takeoff
        t_stance = t_stance(1:takeoff_idx);
        X_stance = X_stance(1:takeoff_idx,:);
        x_takeoff = X_stance(end,:);

        % ----------------- Flight Phase -----------------
        params.q_takeoff = x_takeoff(3:4).';
        params.q_des_flight = params.q_takeoff;

        tspan_flight = [t_stance(end)+0.001, t_stance(end)+3];
        flight_options = odeset('Events', @(t,x) event_touchdown(t,x,params), ...
                                'RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01);
        [t_flight, X_flight] = ode45(@(t,x) dynamics_flight(t,x,params), ...
                                      tspan_flight, x_takeoff, flight_options);

        % ----------------- Impact Phase -----------------
        x_touchdown = X_flight(end,:).';
        x_plus = dynamics_impact(x_touchdown, params);

        % ----------------- Re-Stabilize -----------------
        params.pCOMy_d = 0.2;
        tspan_stabilize = [t_flight(end)+0.001, t_flight(end)+5];
        [t_stabilize, X_stabilize] = ode45(@(t,x) leg_ode(t,x,params), ...
                                            tspan_stabilize, x_plus, stance_options);

        % ----------------- Concatenate -----------------
        if hop == 1
            t_offset = 0;
        else
            t_offset = t_all(end);
        end
        
        t_stance = t_stance + t_offset;
        t_flight = t_flight + t_offset;
        t_stabilize = t_stabilize + t_offset;
        
        t_all = [t_all; t_stance; t_flight; t_stabilize];
        X_all = [X_all; X_stance; X_flight; X_stabilize];

        % Prepare next hop initial state
        x0 = X_stabilize(end,:).';

        % Optional: early exit if it fails to leave ground
        if max(yCOM_stance) < 0.3
            disp("Hopping sequence terminated early (no lift).");
            break;
        end
    end

    % ---------------- Plot & Animate ----------------
    figure; hold on;
    plot(t_all, X_all(:,2), 'LineWidth', 2);
    xlabel('Time (s)'); ylabel('Vertical COM position (m)');
    title('Repeated Hopping (10 cycles)');
    grid on;

    % Remove duplicate or non-monotonic time entries
    [t_all, unique_idx] = unique(t_all, 'stable');
    X_all = X_all(unique_idx, :);
    
    % Now interpolate safely
    fps = 60;
    t_uniform = linspace(t_all(1), t_all(end), fps * (t_all(end) - t_all(1)));
    X_uniform = interp1(t_all, X_all, t_uniform, 'linear', 'extrap');
    
    animate_leg(t_uniform, X_uniform, params);
end
