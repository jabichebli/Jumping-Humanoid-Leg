function simulate_leg_taking_off()

clearvars; close all; clc;

    % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0;
    params.g  = 9.81; 
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);
    params.Kp = 50; params.Kd = 10;
    params.pCOMy_d = 0.2; % desired COM height while stabilizing

    % integration tolerances
    optsFlight = odeset('Events', @(t,x) event_touchdown(t,x,params), 'RelTol',1e-8,'AbsTol',1e-9);
    optsStance  = odeset('Events', @(t,x) event_takeoff(t,x,params), 'RelTol',1e-8,'AbsTol',1e-9);

    % Initial state: [x, y, q1, q2, q3, xdot, ydot, q1dot, q2dot, q3dot] (flight)
    x0 = [0; 0.5; 0.8; 1.2; 0; 0; -0.8; 0; 0; 0]; % example: some downward ydot so touchdown happens

    tNow = 0;
    tFinal = 5.0;

    % Storage for whole sim
    T_all = []; X_all = [];

    % ---------- 1) Flight phase until touchdown ----------
    if tNow < tFinal
        tspan = [tNow, tFinal];
        [t_flight, X_flight, te, ye, ie] = ode45(@(t,x) dynamics_flight(t,x,params), tspan, x0, optsFlight);

        % append
        T_all = [T_all; t_flight];
        X_all = [X_all; X_flight];

        % if touchdown happened, te,ye non-empty
        if isempty(te)
            fprintf('No touchdown detected during flight.\n');
            plot_and_anim(T_all, X_all, params); return;
        end
    else
        plot_and_anim(T_all, X_all, params); return;
    end

    % We have touchdown at last event
    t_td = te(end);
    x_td = ye(end, :)';     % state at touchdown (may have small penetration)
    % fprintf('Touchdown detected at t=%.6f, yFoot=%.6f\n', t_td, auto_pfoot(params.l1, params.l2, x_td(3), x_td(4), x_td(1), x_td(2))(2));

    % ---------- 2) Impact / projection to enforce contact (position and velocity) ----------
    % Project coordinates so foot is exactly on ground (pfoot_y = 0)
    Jst = auto_Jst(params.l1, params.l2, x_td(3), x_td(4));
    pfoot = auto_pfoot(params.l1, params.l2, x_td(3), x_td(4), x_td(1), x_td(2));   % [px; py]
    y_error = pfoot(2);   % if >0 foot above ground, <0 penetrated
    % project the hip vertical coordinate y to remove the y_error:
    x_proj = x_td;          % q = [x;y;q1;q2;q3]
    x_proj(2) = x_proj(2) - y_error;   % shift hip y so foot y = 0

    % Project velocities to satisfy Jst * dq = 0 (no slip into ground at impact)
    dq = x_td(6:10);
    % compute correction: dq_new = dq - Jst' * ((Jst*Jst') \ (Jst*dq))
    % note: if Jst*Jst' is nearly singular, add small regularization
    M = Jst * Jst';
    if rcond(M) < 1e-12
        M = M + 1e-9 * eye(size(M));
    end
    dq_corr = dq - Jst.' * (M \ (Jst * dq));
    x_proj(6:10) = dq_corr;

    % set initial conditions for stance
    x0_stance = x_proj;
    t0_stance = t_td;

    % append a small time (avoid solver thinking event still active at same time)
    tNow = t0_stance;

    % ---------- 3) Stance integration with takeoff event ----------
    % Use ode15s (good for DAEs / stiff) and the stance dynamics that returns constraint forces
    [t_stance, X_stance, te_st, ye_st, ie_st] = ode15s(@(t,x) dynamics_stance(t,x,params), [t0_stance, tFinal], x0_stance, optsStance);

    % append stance to full history (remove first row because same time as last flight row)
    % but careful with duplicates: if times overlap, drop duplicate time
    if ~isempty(t_stance)
        % drop duplicate initial point to avoid repeating row
        T_all = [T_all; t_stance(2:end)];
        X_all = [X_all; X_stance(2:end,:)];
    end

    % check if takeoff event occurred
    if ~isempty(te_st)
        t_to = te_st(end);
        x_to = ye_st(end, :)';
        fprintf('Takeoff detected at t=%.6f\n', t_to);

        % For takeoff, no special reset needed for positions (foot is leaving),
        % but ensure velocities are consistent: when leaving contact, we keep generalized velocities as-is.
        x_post_flight = x_to;  % continue flight with this state

        % Continue flight if desired (single hop) - optional:
        % integrate flight again for remainder of time
        [t_f2, X_f2] = ode45(@(t,x) dynamics_flight(t,x,params), [t_to tFinal], x_post_flight);
        T_all = [T_all; t_f2];
        X_all = [X_all; X_f2];
    end

    % ---------- 4) Plot / animate ----------
    plot_and_anim(T_all, X_all, params);

end

%% small helper for plotting/animation (keeps main function tidy)
function plot_and_anim(T_all, X_all, params)
    figure(1); clf;
    subplot(2,2,1); hold on;
    plot(T_all, X_all(:,3),'LineWidth',2); plot(T_all, X_all(:,4),'LineWidth',2); plot(T_all, X_all(:,5),'LineWidth',2); legend('q1','q2','q3');
    subplot(2,2,2); hold on;
    plot(T_all, X_all(:,8),'LineWidth',2); plot(T_all, X_all(:,9),'LineWidth',2); plot(T_all, X_all(:,10),'LineWidth',2); legend('dq1','dq2','dq3');
    subplot(2,2,3); hold on;
    plot(T_all, X_all(:,1),'LineWidth',2); plot(T_all, X_all(:,2),'LineWidth',2); legend('x','y');
    subplot(2,2,4); hold on;
    plot(T_all, X_all(:,6),'LineWidth',2); plot(T_all, X_all(:,7),'LineWidth',2); legend('dx','dy');

    % animate if you have animate_leg function
    try
        animate_leg(T_all, X_all, params);
    catch
        fprintf('animate_leg not found or failed; skipping animation.\n');
    end
end
