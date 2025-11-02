% event_takeoff.m
function [value, isterminal, direction] = event_takeoff1(t, x, params)
    % -------------------------------------------------------------------------
    % Detects takeoff when the vertical ground reaction force (Fy) drops to
    % zero, with added robustness checks.
    % -------------------------------------------------------------------------

    % % --- Safety Checks ---
    % % 1. Do not allow takeoff before the hold/squat phase is complete.
    % min_takeoff_time = params.T_hold;
    % if t < min_takeoff_time
    %     value = 1;      % Stay positive, do not trigger
    %     isterminal = 1;
    %     direction = 0;
    %     return;
    % end
    % 
    % % 2. Check that the center of mass is moving upwards.
    % dq = x(6:10);
    % Jh_com = auto_JpCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,x(3),x(4),x(5));
    % dpCOM_actual = Jh_com * dq;
    % min_takeoff_vel = 0.1; % Must have some upward velocity (m/s)
    % if dpCOM_actual(2) < min_takeoff_vel
    %     value = 1;      % Stay positive, do not trigger
    %     isterminal = 1;
    %     direction = 0;
    %     return;
    % end
    
    % --- Primary Trigger Condition ---
    % If safety checks pass, monitor the vertical ground force.
    % We must re-calculate the dynamics to get the force for the current state.
    [~, ~, lambda] = dynamics_stance1(t, x, params);
    Fy = lambda(2);
    
    value = Fy;          % The value that ODE45 monitors. Event triggers when this is zero.
    isterminal = 1;      % Stop the simulation when the event occurs.
    direction = -1;      % Trigger only when the value is decreasing (going from + to 0).
end