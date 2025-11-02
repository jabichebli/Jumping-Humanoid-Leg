% -------------------------------------------------------------------------
% event_touchdown.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function [value, isterminal, direction] = event_touchdown(~, x, params)
    % ---------------------------------------------------------------------
    % Event function used by ODE45 to detect when the leg makes contact
    % with the ground during simulation. It computes the vertical position
    % of the foot and triggers when that position crosses zero (touchdown).
    %
    % Inputs:
    %   x       - current state vector [q; dq]
    %   params  - structure containing model parameters (e.g., link lengths)
    %
    % Outputs:
    %   value       - event function value; ODE45 stops when this crosses zero
    %   isterminal  - 1 → stop integration at the event
    %   direction   - -1 → detect only when value decreases through zero
    % ---------------------------------------------------------------------

    % Extract generalized coordinates
    q = x(1:5);
    x_hip = q(1);
    y_hip = q(2);
    q1 = q(3);
    q2 = q(4);

    % Retrieve link lengths
    l1 = params.l1;
    l2 = params.l2;

    % Compute absolute position of the foot using kinematics
    pFoot = auto_pfoot(l1, l2, q1, q2, x_hip, y_hip);
    yFoot = pFoot(2);  % Extract vertical position of foot

    % Define event conditions for ODE45
    value = yFoot;        % Event triggers when foot height = 0
    isterminal = 1;       % Stop integration upon touchdown
    direction = -1;       % Detect only downward crossings (approach ground)
end