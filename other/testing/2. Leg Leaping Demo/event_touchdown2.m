% event_touchdown2.m
function [value, isterminal, direction] = event_touchdown2(t, x, params)
    % -------------------------------------------------------------------------
    % Detects touchdown when the foot's vertical position reaches zero.
    % (This file is identical to event_touchdown1.m)
    % -------------------------------------------------------------------------

    q = x(1:5);
    x_hip = q(1); y_hip = q(2); q1 = q(3); q2 = q(4);
    l1 = params.l1; l2 = params.l2;

    pFoot = auto_pfoot(l1, l2, q1, q2, x_hip, y_hip);
    yFoot = pFoot(2);

    value = yFoot;        % The value that ODE45 monitors.
    isterminal = 1;       % Stop the simulation.
    direction = -1;       % Trigger only when decreasing.
end