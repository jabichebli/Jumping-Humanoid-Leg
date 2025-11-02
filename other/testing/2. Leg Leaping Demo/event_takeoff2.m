% event_takeoff2.m
function [value, isterminal, direction] = event_takeoff2(t, x, params)
    % -------------------------------------------------------------------------
    % Detects takeoff when the vertical ground reaction force (Fy) drops to
    % zero. This version calls dynamics_stance2 to get the force.
    % -------------------------------------------------------------------------
    
    % --- Primary Trigger Condition ---
    % We must re-calculate the dynamics to get the force for the current state.
    [~, ~, lambda] = dynamics_stance2(t, x, params);
    Fy = lambda(2);
    
    value = Fy;          % The value that ODE45 monitors.
    isterminal = 1;      % Stop the simulation.
    direction = -1;      % Trigger only when the value is decreasing.
end