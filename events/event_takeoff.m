% -------------------------------------------------------------------------
% event_takeoff.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function [value, isterminal, direction] = event_takeoff(t, x, params)
    % ---------------------------------------------------------------------
    % Event function used by ODE45 to detect the instant of takeoff during
    % simulation. Takeoff occurs when the vertical ground reaction force (Fy)
    % becomes zero, indicating the leg has lost contact with the ground.
    %
    % Inputs:
    %   t       - current simulation time
    %   x       - current state vector [q; dq]
    %   params  - structure containing model parameters used by dynamics_stance2()
    %
    % Outputs:
    %   value       - event function value; ODE45 stops when this crosses zero
    %   isterminal  - 1 → stop integration at the event
    %   direction   - -1 → detect only when value decreases through zero
    % ---------------------------------------------------------------------

    % Compute stance-phase dynamics to obtain contact forces
    [~, ~, lambda] = dynamics_stance(t, x, params);

    % Extract vertical ground reaction force
    Fy = lambda(2);

    % Define event conditions for ODE45
    value = Fy;           % Event triggers when vertical force = 0
    isterminal = 1;       % Stop integration upon takeoff
    direction = -1;       % Detect only downward zero-crossing (force vanishes)
end
