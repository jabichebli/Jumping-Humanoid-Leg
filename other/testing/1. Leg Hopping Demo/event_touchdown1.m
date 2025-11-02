% event_touchdown.m
function [value, isterminal, direction] = event_touchdown1(t, x, params)
    % -------------------------------------------------------------------------
    % Detects touchdown when the foot's vertical position reaches zero.
    % -------------------------------------------------------------------------

    q = x(1:5);
    dq = x(6:10);
    x_hip = q(1);
    y_hip = q(2);
    q1 = q(3);
    q2 = q(4);

    l1 = params.l1;
    l2 = params.l2;

    % --- Primary Trigger Condition: Foot Height ---
    pFoot = auto_pfoot(l1, l2, q1, q2, x_hip, y_hip);
    yFoot = pFoot(2);

    % % --- Safety Check: Foot must be moving downward ---
    % J_foot = auto_Jpfoot(l1,l2,q1,q2);
    % d_pFoot = J_foot*dq;
    % dyFoot = d_pFoot(2);
    % 
    % if dyFoot > 0
    %     value = 1;      % Stay positive if moving up, do not trigger
    %     isterminal = 1;
    %     direction = 0;
    %     return;
    % end

    value = yFoot;        % The value that ODE45 monitors.
    isterminal = 1;       % Stop the simulation.
    direction = -1;       % Trigger only when decreasing (moving from air to ground).
end